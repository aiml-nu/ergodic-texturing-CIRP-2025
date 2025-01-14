import jax.numpy as jnp
import functions
import ergodic_controller

class simulation():
    def __init__(self, 
                 phi__N_N: jnp.array,
                 domain_size: float = 250.0,
                 num_basis: int = 10 or None, 
                 velocity_limit: float = 75.0,
                 wheel_spacing: float = 29.6,
                 mpc_horizon: int = 10,
                 mpc_dt: float = 0.1,
                 mpc_step: float = 1.0,
                 barrier_coefficient: float = 1.0,
                 control_coefficient: float = 1.0,
                 ergodic_coefficient: float = 1.0,
                 sim_dt: float = 0.1):
        self.sim_dt = sim_dt
        self.controller = ergodic_controller.ergodic_controller(phi__N_N,
                                                                domain_size,
                                                                num_basis, 
                                                                velocity_limit,
                                                                wheel_spacing,
                                                                mpc_horizon,
                                                                mpc_dt,
                                                                mpc_step,
                                                                barrier_coefficient,
                                                                control_coefficient,
                                                                ergodic_coefficient)

    def run_simulation(self,
                       time_steps: int,
                       save_every: int,
                       dimple_every: int,
                       x_n_0__3: jnp.array):
        x_traj = []
        u_traj = []
        t_traj = []
        barrier_cost_traj = []
        control_cost_traj = []
        ergodic_cost_traj = []
        dimple_traj = []

        self.controller.reset()
        x_n__3 = x_n_0__3
        t = 0
        for k in range(int(time_steps)):
            self.controller.update_control_horizon(x_n__3, t) 
            u_n__2 = self.controller.get_current_control_normalized()
            x_n__3 = functions._x_inc__3(x_n__3, u_n__2, self.controller.s_n, self.sim_dt) 
            t = t + self.sim_dt 
            self.controller.update_distribution_time(x_n__3, t, self.sim_dt) 
            if (k % save_every == 0):
                print("Progress: {}%".format(100*k/time_steps))
                x_traj.append(x_n__3.tolist())
                u_traj.append(u_n__2.tolist())
                t_traj.append(t)
                barrier_cost_traj.append(self.controller.get_cost_barrier(x_n__3))
                control_cost_traj.append(self.controller.get_cost_control(u_n__2))
                ergodic_cost_traj.append(self.controller.get_cost_ergodic())
            if (k % dimple_every == 0):
                self.controller.update_distribution_dimple(x_n__3, t, dimple_every * self.sim_dt)
                dimple_traj.append([x_n__3[0], x_n__3[1]])
        return (jnp.array(dimple_traj),
                jnp.array(x_traj),
                jnp.array(u_traj),
                jnp.array(t_traj),
                jnp.array(barrier_cost_traj),
                jnp.array(control_cost_traj),
                jnp.array(ergodic_cost_traj))

    def get_reconstruction_target(self):
        return self.controller.get_reconstruction_target()

    def get_reconstruction_time(self):
        return self.controller.get_reconstruction_time()
    
    def get_reconstruction_dimples(self):
        return self.controller.get_reconstruction_dimple()