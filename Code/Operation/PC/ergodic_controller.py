import functions
import jax.numpy as jnp

class ergodic_controller():
    def __init__(self, 
                 phi__N_N: jnp.array,
                 domain_size: float = 250.0, 
                 num_basis: int = 10 or None, 
                 velocity_limit: float = 75.0, 
                 wheel_spacing: float = 29.6, 
                 mpc_horizon: int = 10, 
                 mpc_dt: float = 0.01, 
                 mpc_step: float = 1.0,
                 barrier_coefficient: float = 1.0,
                 control_coefficient: float = 1.0,
                 ergodic_coefficient: float = 1.0):
        '''
        Generates an ergodic controller for a two-wheel, differential drive robot.
        '''
        # Domain parameters
        self.L = domain_size

        # Robot parameters
        self.u_lim = velocity_limit
        self.u_n_lim = self.u_lim / self.L
        self.s = wheel_spacing 
        self.s_n = self.s / self.L

        # Ergodic control
        self.horizon = mpc_horizon
        self.B_coef = barrier_coefficient
        self.D_coef = control_coefficient
        self.E_coef = ergodic_coefficient
        self.mpc_dt = mpc_dt
        self.step = mpc_step

        # Target distribution
        self.phi__N_N = phi__N_N
        assert phi__N_N.shape[0] == phi__N_N.shape[1]
        self.N = self.phi__N_N.shape[0]
        self.pi__N = (jnp.arange(self.N)+0.5) * jnp.pi / (self.N) # self.N-1
        if num_basis is None:
            self.K = self.N
        else:
            self.K = num_basis
        self.k__K = jnp.arange(self.K,dtype=jnp.float32)
        self.Lambdak = functions._Lambdak__K_K(self.k__K)
        self.k_pi__K = self.k__K * jnp.pi
        self.x_n__N = (jnp.arange(self.N,dtype=jnp.float32)+0.5) / (self.N)
        self.hk__K_K = functions._hk__K_K(self.pi__N, self.k__K)
        self.Fk__N_N_K_K = functions._Fk__N_N_K_K(self.x_n__N, self.k__K, self.hk__K_K)
        self.phik__K_K = functions._phik__K_K(self.phi__N_N, self.Fk__N_N_K_K)

        # Result distributions
        self.Lambdak__K_K = functions._Lambdak__K_K(self.k__K)
        self.ck__K_K = jnp.zeros((self.K,self.K),dtype=jnp.float32) 
        self.ck_dimp__K_K = jnp.zeros((self.K,self.K),dtype=jnp.float32)

        # Barrier
        self.B__N_N = functions._generate_B__N_N(phi__N_N)
        self.dB__N_N_2 = functions._generate_dB__N_N_2(self.B__N_N)
        self.dB__N_N = functions._generate_dB__N_N(self.dB__N_N_2)

        # Control horizon
        self.u_n_init = jnp.ones(2) * self.u_n_lim * 0.1
        self.u_n__horizon_2 = jnp.repeat(self.u_n_init[None,:],self.horizon,axis=0)

    def get_current_control_normalized(self):
        '''Get the most current control from the horizon.'''
        return self.u_n__horizon_2[0]
    
    def get_current_control_scaled(self):
        '''Get the most current control from the horizon.'''
        return self.u_n__horizon_2[0] * self.L
    
    def convert_position_normalized(self,
                                    x__3):
        '''Take a position measurement and convert it to the normalized version while checking against limits.'''
        eps = 1e-4
        x_n__3 = x__3
        x_n__2 = x__3[:2] / self.L
        condlist = [x_n__2 > jnp.array([1-eps,   jnp.inf]), 
                    x_n__2 > jnp.array([jnp.inf, 1-eps  ]), 
                    x_n__2 < jnp.array([eps,     eps    ])]
        x_n__2 = jnp.piecewise(x_n__2, condlist, [1-eps, 1-eps, eps, lambda q: q])
        x_n__3 = x_n__3.at[:2].set(x_n__2)
        return x_n__3

    def get_cost_control(self, 
                         u_n__2):
        '''Get the cost due to the given control.'''
        return self.D_coef * functions._D(u_n__2, self.u_n_lim)
    
    def get_cost_ergodic(self):
        '''Get the cost due to the ergodic metric.'''
        return self.E_coef * functions._E(self.ck__K_K, self.phik__K_K, self.Lambdak__K_K)

    def get_cost_barrier(self,
                         x_n__3):
        '''Get the cost due to the barrier function.'''
        return self.B_coef * functions._B(x_n__3, self.B__N_N)

    def get_reconstruction_target(self):
        '''Reconstruct the image based on the target distribution.'''
        return functions._phi_recon__N_N(self.phik__K_K, self.Fk__N_N_K_K)
    
    def get_reconstruction_time(self):
        '''Reconstruct the image based on the time distribution.'''
        return functions._phi_recon__N_N(self.ck__K_K, self.Fk__N_N_K_K)
    
    def get_reconstruction_dimple(self):
        '''Reconstruct the image based on the dimple distribution.'''
        return functions._phi_recon__N_N(self.ck_dimp__K_K, self.Fk__N_N_K_K)

    def update_control_horizon(self, 
                               x_n__3, 
                               t):
        '''Take a forward and backward pass of MPC to update the control horizon using the latest measured position.'''
        [x_n__horizon_3,
         ck__horizon_K_K,
         x_n_f__3,
         ck_f__K_K,
         t_f] = functions._forward(x_n_0__3=x_n__3,
                                   ck_0__K_K=self.ck__K_K,
                                   u_n__horizon_2=self.u_n__horizon_2,
                                   hk__K_K=self.hk__K_K,
                                   k_pi__K=self.k_pi__K,
                                   t_0=t,
                                   dt=self.mpc_dt,
                                   s_n=self.s_n,
                                   horizon=self.horizon)
        self.u_n__horizon_2 = functions._backward(x_n_f__3=x_n_f__3,
                                                  ck_f__K_K=ck_f__K_K,
                                                  x_n__horizon_3=x_n__horizon_3,
                                                  u_n__horizon_2=self.u_n__horizon_2,
                                                  ck__horizon_K_K=ck__horizon_K_K,
                                                  hk__K_K=self.hk__K_K,
                                                  phik__K_K=self.phik__K_K,
                                                  Lambdak__K_K=self.Lambdak__K_K,
                                                  k_pi__K=self.k_pi__K,
                                                  dB__N_N=self.dB__N_N,
                                                  t_f=t_f,
                                                  dt=self.mpc_dt,
                                                  step=self.step,
                                                  u_n_lim=self.u_n_lim,
                                                  s_n=self.s_n,
                                                  B_coef=self.B_coef,
                                                  D_coef=self.D_coef,
                                                  E_coef=self.E_coef,
                                                  horizon=self.horizon)

    def update_distribution_time(self,
                                 x_n__3,
                                 t,
                                 dt):
        '''Update the time distribution using the measured state and time.'''
        self.ck__K_K = functions._ck_inc__K_K(self.ck__K_K, x_n__3, self.k_pi__K, self.hk__K_K, t, dt)

    def update_distribution_dimple(self,
                                   x_n__3,
                                   t,
                                   dt):
        '''Update the dimple distribution using the measured state and time.'''
        self.ck_dimp__K_K = functions._ck_inc__K_K(self.ck_dimp__K_K, x_n__3, self.k_pi__K, self.hk__K_K, t, dt)

    def reset(self):
        self.ck__K_K = jnp.zeros((self.K,self.K),dtype=jnp.float32)
        self.ck_dimp__K_K = jnp.zeros((self.K,self.K),dtype=jnp.float32)