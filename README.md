# LidarPF
## State Space Model
### State
$$\mathbf{x}=[x,y,\theta]^T$$
### State Transit function
$$\mathbf{x}_{k+1}=f(\mathbf{x}_k)$$

Due to the limitations of the data, we can't get any useful prediction, so we just use the previous state as the prediction of the next state.
### Measurement
$$\mathbf{z}_k=h(\mathbf{x}_k)+v_k$$

Where the function $$h(\cdot)$$ is how the lidar measures the distance from the car to the wall from -135 to 135 degrees, which is a nonlinear function. $$\mathbf{z}_k$$ is the measurement.
