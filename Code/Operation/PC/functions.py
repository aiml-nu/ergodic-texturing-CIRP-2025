import jax.numpy as jnp
from jax import jit, vmap
from functools import partial
from matplotlib.image import imread

# Note that "Normalized" is taken to mean the real value is divided by the side length of the domain.
# Everything will be square to keep things simple

# Custom Arc

def _generate_arc(size: int,
                  width: int,
                  min:  float,
                  max:  float
                  ) ->  jnp.array:
    ij = jnp.stack(jnp.meshgrid(jnp.arange(size),jnp.arange(size),indexing='ij'),axis=-1)
    angles = (jnp.square(4*jnp.atan2(ij[:,:,0],ij[:,:,1])/jnp.pi - 1) * (max - min)) + min
    distances = jnp.sqrt(jnp.sum(jnp.square(ij - jnp.zeros(2)[None,None,:]),axis=-1))
    points = angles * (distances > (size-width))
    points = points * (distances < size)
    dxdy = size**2
    phi = dxdy * points / jnp.sum(points) 
    return phi

# Barrier Generation

@jit
def _get_outside_edges(binary_image: jnp.array
                       ) ->          jnp.array:
    '''Takes an image composed of either 0 or 1 and gets the outside edges.'''
    left_edges =           jnp.concatenate((jnp.zeros((1,binary_image.shape[1])),jnp.where(jnp.diff(binary_image,axis=0)<0, 1, 0)),axis=0)
    right_edges = jnp.flip(jnp.concatenate((jnp.zeros((1,binary_image.shape[1])),jnp.where(jnp.diff(jnp.flip(binary_image,axis=0),axis=0)<0, 1, 0)),axis=0),axis=0)
    up_edges =             jnp.concatenate((jnp.zeros((binary_image.shape[0],1)),jnp.where(jnp.diff(binary_image,axis=1)<0, 1, 0)),axis=1)
    down_edges =  jnp.flip(jnp.concatenate((jnp.zeros((binary_image.shape[0],1)),jnp.where(jnp.diff(jnp.flip(binary_image,axis=1),axis=1)<0, 1, 0)),axis=1),axis=1)
    outside_edges = jnp.logical_or(left_edges.astype(bool), 
                                    jnp.logical_or(right_edges.astype(bool), 
                                                    jnp.logical_or(up_edges.astype(bool), 
                                                                    down_edges.astype(bool))))
    return outside_edges

def _expand_outside_edges(phi__N_N: jnp.array
                          ) ->      jnp.array:
    '''Binarizes and expands the outside edges of an image to its borders.'''
    binary_image = jnp.where(phi__N_N > 1e-8, 1, 0)
    expanded_outside_edges = binary_image
    i = 1
    while (jnp.any(binary_image < 1e-8)):
        outside_edges = _get_outside_edges(binary_image)
        expanded_outside_edges = expanded_outside_edges + (i+1)*outside_edges
        binary_image = jnp.where(expanded_outside_edges > 0.001, 1, 0)
        i = i + 1
    return expanded_outside_edges

def _generate_B__N_N(phi__N_N: jnp.array
                     ) ->      jnp.array:
    return _expand_outside_edges(phi__N_N)

@jit
def _generate_dB__N_N_2(B__N_N: jnp.array
                        ) ->    jnp.array:
    N = B__N_N.shape[0]
    dB__N_N_2 = (jnp.stack([jnp.concatenate((jnp.diff(B__N_N,axis=0),jnp.zeros((1,N),dtype=jnp.int32)),axis=0),
                            jnp.concatenate((jnp.diff(B__N_N,axis=1),jnp.zeros((N,1),dtype=jnp.int32)),axis=1)],axis=-1)+
                 jnp.stack([jnp.concatenate((jnp.zeros((1,N),dtype=jnp.int32),jnp.diff(B__N_N,axis=0)),axis=0),
                            jnp.concatenate((jnp.zeros((N,1),dtype=jnp.int32),jnp.diff(B__N_N,axis=1)),axis=1)],axis=-1))
    return dB__N_N_2

@jit
def _generate_dB__N_N(dB__N_N_2: jnp.array
                      ) ->       jnp.array:
    dB__N_N = dB__N_N_2[:,:,0] + 2 + 10 * (dB__N_N_2[:,:,1] + 2) # Encode directions in integers
    return dB__N_N

# Dynamics

@jit
def _f__3(x_n__3: jnp.array, # Normalized position
          u_n__2: jnp.array, # Normalized control
          s_n:    float      # Normalized wheel spacing
          ) ->    jnp.array: 
    '''Two-wheel robot dynamics.'''
    return jnp.array([jnp.cos(x_n__3[2])*(u_n__2[0]+u_n__2[1])/2, 
                      jnp.sin(x_n__3[2])*(u_n__2[0]+u_n__2[1])/2, 
                      (-u_n__2[0]+u_n__2[1])/s_n]) 

@jit
def _dfdx__3_3(x_n__3: jnp.array, # Normalized position
               u_n__2: jnp.array  # Normalized control
               ) ->  jnp.array:
    '''Gradient of robot dynamics with respect to the state.'''
    return jnp.array([[0, 0, -jnp.sin(x_n__3[2])*(u_n__2[0]+u_n__2[1])/2],
                      [0, 0,  jnp.cos(x_n__3[2])*(u_n__2[0]+u_n__2[1])/2],
                      [0, 0,  0]])

@jit
def _dfdu__3_2(x_n__3: jnp.array, # Normalized position
               s_n:    float      # Normalized wheel spacing
               ) ->    jnp.array: 
    '''Gradient of robot dynamics with respect to the control.'''
    return jnp.array([[jnp.cos(x_n__3[2])/2, jnp.cos(x_n__3[2])/2],
                      [jnp.sin(x_n__3[2])/2, jnp.sin(x_n__3[2])/2],
                      [-1/s_n,               1/s_n]])

# Increment State

@jit
def _x_inc__3(x_n__3: jnp.array, # Normalized position
              u_n__2: jnp.array, # Normalized control
              s_n:    float,     # Normalized spacing
              dt:     float      # Time step
              ) ->    jnp.array:
    '''Increment the state while taking into account the limits on the domain.'''
    eps = 1e-4
    x_n__3 = x_n__3 + _f__3(x_n__3, u_n__2, s_n) * dt
    x_n__2 = x_n__3[:2]
    condlist = [x_n__2 > jnp.array([1-eps,   jnp.inf]), 
                x_n__2 > jnp.array([jnp.inf, 1-eps  ]), 
                x_n__2 < jnp.array([eps,     eps    ])]
    x_n__2 = jnp.piecewise(x_n__2, condlist, [1-eps, 1-eps, eps, lambda q: q])
    x_n__3 = x_n__3.at[:2].set(x_n__2)
    return x_n__3

# Decrement Control

@jit
def _u_dec__2(u_n__2:  jnp.array, # Normalized control
              du_n__2: jnp.array, # Change in normalized control
              step:    float,     # Increment step
              u_n_lim: float      # Normalized control limit
              ) ->   jnp.array:
    '''Decrement the controls while taking into account their limits.'''
    u_n__2 = u_n__2 - du_n__2 * step
    condlist = [u_n__2 >  u_n_lim,
                u_n__2 < -u_n_lim]
    return jnp.piecewise(u_n__2, condlist, [u_n_lim, -u_n_lim, lambda q: q])

# Control cost

@jit
def _D(u_n__2:  jnp.array,
       u_n_lim: float
       ) ->     float:
    '''Evaluate the control cost.'''
    return 0.5*jnp.sum(jnp.square(u_n__2/u_n_lim))

@jit
def _dD__2(u_n__2:  jnp.array,
           u_n_lim: float
           ) ->     jnp.array:
    '''Evaluate the gradient of the control cost with respect to the control.'''
    return u_n__2/u_n_lim

# Barrier cost

@jit
def _B(x_n__3: jnp.array,
       B__N_N: jnp.array
       ) ->    float:
    '''Calculate the value of the barrier function at a single state.'''
    return B__N_N[jnp.int32(B__N_N.shape[0]*x_n__3[0]),
                  jnp.int32(B__N_N.shape[1]*x_n__3[1])]

@jit
def _dB__3(x_n__3:    jnp.array,
           dB__N_N: jnp.array
           ) ->     jnp.array:
    '''Calculate the gradient of the barrier function at a single state.'''
    code =  dB__N_N[jnp.int32(dB__N_N.shape[0]*x_n__3[0]),
                    jnp.int32(dB__N_N.shape[1]*x_n__3[1])]
    dBx = jnp.mod(code, 10) - 2
    dBy = jnp.floor_divide(code, 10) - 2
    dB__3 = jnp.array([dBx, dBy, 0])
    return dB__3

# Ergodic cost

@jit
def _E(ck__K_K:      jnp.array,
       phik__K_K:    jnp.array,
       Lambdak__K_K: jnp.array
       ) ->          float:
    '''Evaluate the ergodic metric.'''
    E = jnp.sum(Lambdak__K_K * (ck__K_K - phik__K_K)**2)
    return E

@jit
def _dE__3(x_n__3:       jnp.array,
           ck_old__K_K:  jnp.array,
           phik__K_K:    jnp.array,
           Lambdak__K_K: jnp.array,
           k_pi__K:      jnp.array,
           hk__K_K:      jnp.array,
           t:            float,
           dt:           float
           ) ->          tuple[jnp.array, jnp.array]:
    '''Evaluate the gradient of the ergodic metric for a single point placement.'''
    x = x_n__3[0] * k_pi__K
    y = x_n__3[1] * k_pi__K
    cosx__K = jnp.cos(x)
    sinx__K = jnp.sin(x)
    cosy__K = jnp.cos(y)
    siny__K = jnp.sin(y)
    hk_inv__K_K = 1/hk__K_K # dt/(hk__K_K * (t + dt))
    ck_new__K_K = _ck_inc__K_K(ck_old__K_K, x_n__3, k_pi__K, hk__K_K, t, dt)
    diff__K_K = Lambdak__K_K * (ck_new__K_K - phik__K_K)
    dcdx__K_K = -hk_inv__K_K * sinx__K[:,None] * cosy__K[None,:] * k_pi__K[:,None] 
    dcdy__K_K = -hk_inv__K_K * siny__K[None,:] * cosx__K[:,None] * k_pi__K[None,:]
    dEdx = 2 * jnp.sum(diff__K_K * dcdx__K_K)
    dEdy = 2 * jnp.sum(diff__K_K * dcdy__K_K)
    dE__3 = jnp.array([dEdx,dEdy,0])
    return dE__3

@jit
def _Lambdak__K_K(k__K: jnp.array,
                  ) ->   jnp.array:
    '''Normalizing term for the ergodic metric.'''
    Lambdak__K_K = (1 + (k__K**2)[:,None] + (k__K**2)[None,:])**-1.5
    return Lambdak__K_K

@jit
def _ck_inc__K_K(ck_old__K_K: jnp.array,
                 x_n__3:      jnp.array,
                 k_pi__K:     jnp.array,
                 hk__K_K:     jnp.array,
                 t:           float,
                 dt:          float
                 ) ->         jnp.array:
    '''Increment ck using the time.'''
    Fk__K_K = _Fk__K_K(x_n__3, k_pi__K, hk__K_K)
    ck_new__K_K = (t * ck_old__K_K + dt * Fk__K_K) / (t + dt)
    return ck_new__K_K

@jit
def _Fk__K_K(x_n__3:  jnp.array,
             k_pi__K: jnp.array,
             hk__K_K: jnp.array
             ) ->     jnp.array:
    '''Calculate the Fourier bases for a single point.'''
    x = x_n__3[0] * k_pi__K
    y = x_n__3[1] * k_pi__K
    cosx__K = jnp.cos(x)
    cosy__K = jnp.cos(y)
    Fk__K_K = cosx__K[:,None] * cosy__K[None,:] / hk__K_K
    return Fk__K_K

@jit
def _Fk__N_N_K_K(x_n__N:  jnp.array,
                 k__K:    jnp.array,
                 hk__K_K: jnp.array
                 ) ->     jnp.array:
    '''Calculate the Fourier bases for a 2D collection of points.'''
    cos__N_K = jnp.cos(k__K[None,:] * jnp.pi * x_n__N[:,None])
    Fk__N_N_K_K =  (1/hk__K_K)[None,None,:,:] * cos__N_K[:,None,:,None] * cos__N_K[None,:,None,:]
    return Fk__N_N_K_K

@jit
def _phik__K_K(phi__N_N:    jnp.array,
               Fk__N_N_K_K: jnp.array       
               ) ->         jnp.array:
    '''Calculate phik from an input distribution phi.'''
    dxdy = (Fk__N_N_K_K.shape[0])**2
    phik__K_K = jnp.sum(phi__N_N[:,:,None,None] * Fk__N_N_K_K, (0,1)) / dxdy
    return phik__K_K

@jit
def _phi_recon__N_N(anyk__K_K:   jnp.array,
                    Fk__N_N_K_K: jnp.array
                    ) ->         jnp.array:
    '''Reconstruct the target image phi from a 2D Fourier basis description (either phik or ck).'''
    phi_recon__N_N = jnp.sum(anyk__K_K[None,None,:,:] * Fk__N_N_K_K, (2,3))
    return phi_recon__N_N

def _phi_img__N_N(filepath: str
                  ) -> jnp.array:
    '''Read the target image phi from an image file.'''
    phi = 1 - jnp.array(imread(filepath))
    assert phi.shape[0] == phi.shape[1]
    phi = jnp.swapaxes(phi,0,1)
    phi = jnp.flip(phi,axis=1)
    dxdy = phi.shape[0]**2 
    phi_img__N_N = dxdy * phi / jnp.sum(phi, (0,1))  # Normalize
    return phi_img__N_N

@jit
def _hk__K_K(pi__N: jnp.array,
             k__K:  jnp.array,
             ) ->   jnp.array:
    '''Calculate hk for a given input image size.'''
    cos2__N_K = jnp.square(jnp.cos(pi__N[:,None] * k__K[None,:]))
    hk__K_K = jnp.sqrt(jnp.sum(cos2__N_K[:,None,:,None] * cos2__N_K[None,:,None,:], (0,1))) / (pi__N.shape[0])
    return hk__K_K

@jit
def _l(x_n__3:       jnp.array, 
       u_n__2:       jnp.array, 
       ck__K_K:      jnp.array, 
       phik__K_K:    jnp.array,
       Lambdak__K_K: jnp.array,
       B__N_N:       jnp.array,
       u_n_lim:      float,
       B_coef:       float,
       D_coef:       float,
       E_coef:       float
       ) ->          float:
    '''Complete objective function.'''
    return (B_coef * _B(x_n__3, B__N_N) + 
            D_coef * _D(u_n__2, u_n_lim) + 
            E_coef * _E(ck__K_K, phik__K_K, Lambdak__K_K))

@jit
def _dldx__3(x_n__3:       jnp.array, 
             ck__K_K:      jnp.array, 
             hk__K_K:      jnp.array,
             phik__K_K:    jnp.array,
             Lambdak__K_K: jnp.array,
             k_pi__K:      jnp.array,
             dB__N_N:      jnp.array,
             t:            float, 
             dt:           float,
             B_coef:       float,
             E_coef:       float
             ) ->          jnp.array:
    '''Gradient of the complete objective function with respect to the state.'''
    return (B_coef * _dB__3(x_n__3, dB__N_N) +
            E_coef * _dE__3(x_n__3, ck__K_K, phik__K_K, Lambdak__K_K, k_pi__K, hk__K_K, t, dt))

@jit
def _dldu__2(u_n__2:   jnp.array,
             u_n_lim:  float,
             D_coef:   float
             ) ->      jnp.array:
    '''Gradient of the complete objective function with respect to the control.'''
    return D_coef * _dD__2(u_n__2, u_n_lim)

@partial(jit,static_argnames=['horizon'])
def _forward(x_n_0__3:       jnp.array,
             ck_0__K_K:      jnp.array,
             u_n__horizon_2: jnp.array,
             hk__K_K:        jnp.array,
             k_pi__K:        jnp.array,
             t_0:            float,
             dt:             float,
             s_n:            float,
             horizon:        int
             ) ->            tuple[jnp.array, jnp.array, jnp.array, jnp.array, float]:
    '''Run a forward pass of Model Predictive Control.'''
    x_n__horizon_3 = jnp.zeros((horizon,3))
    K = hk__K_K.shape[0]
    ck__horizon_K_K = jnp.zeros((horizon,K,K))
    x_n__3 = x_n_0__3
    ck__K_K = ck_0__K_K
    t = t_0
    for i in range(horizon):
        x_n__horizon_3 = x_n__horizon_3.at[i].set(x_n__3)
        ck__horizon_K_K = ck__horizon_K_K.at[i].set(ck__K_K)
        u_n__2 = u_n__horizon_2[i]
        t = t + dt
        x_n__3 = _x_inc__3(x_n__3, u_n__2, s_n, dt)
        ck__K_K = _ck_inc__K_K(ck__K_K, x_n__3, k_pi__K, hk__K_K, t, dt)
    return x_n__horizon_3, ck__horizon_K_K, x_n__3, ck__K_K, t

@partial(jit,static_argnames=['horizon'])
def _backward(x_n_f__3:        jnp.array,
              ck_f__K_K:       jnp.array,
              x_n__horizon_3:  jnp.array,
              u_n__horizon_2:  jnp.array,
              ck__horizon_K_K: jnp.array,
              hk__K_K:         jnp.array,
              phik__K_K:       jnp.array,
              Lambdak__K_K:    jnp.array,
              k_pi__K:         jnp.array,
              dB__N_N:         jnp.array,
              t_f:             float,
              dt:              float, 
              step:            float,
              u_n_lim:         float,
              s_n:             float,
              B_coef:          float,
              D_coef:          float,
              E_coef:          float,
              horizon:         int
              ) ->             jnp.array:
    '''Run a backward pass of Model Predictive Control.'''
    t = t_f
    x_n__3 = x_n_f__3
    ck__K_K = ck_f__K_K
    p__3 = jnp.zeros(3)
    for i in range(horizon):
        u_n__2 = u_n__horizon_2[horizon-i-1]
        dfdx__3_3 = _dfdx__3_3(x_n__3, u_n__2)
        dldx__3 = _dldx__3(x_n__3, ck__K_K, hk__K_K, phik__K_K, Lambdak__K_K, k_pi__K, dB__N_N, t, dt, B_coef, E_coef)
        p_dot__3 = (-jnp.dot(dfdx__3_3.T, p__3) - dldx__3)
        p__3 = p__3 - p_dot__3 * dt
        dfdu__3_2 = _dfdu__3_2(x_n__3, s_n)
        dldu__2 = _dldu__2(u_n__2, u_n_lim, D_coef)
        du_n__2 = jnp.dot(dfdu__3_2.T, p__3) + dldu__2
        u_n__2 = _u_dec__2(u_n__2, du_n__2, step, u_n_lim)
        u_n__horizon_2 = u_n__horizon_2.at[horizon-i-1].set(u_n__2)
        t = t - dt
        x_n__3 = x_n__horizon_3[horizon-i-1]
        ck__K_K = ck__horizon_K_K[horizon-i-1]
    return u_n__horizon_2
