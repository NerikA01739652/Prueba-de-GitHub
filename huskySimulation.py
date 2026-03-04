import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time

def setup_environment():
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # Create red block
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2], rgbaColor=[1, 0, 0, 1])
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
    block_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual, basePosition=[2, 1, 0.2])

    car = p.loadURDF("husky/husky.urdf", basePosition=[0, 1, 0.1])
    return car, block_id

def get_camera_image(car):
    car_pos, car_orn = p.getBasePositionAndOrientation(car)
    rot_matrix = p.getMatrixFromQuaternion(car_orn)
    forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]

    cam_pos = [car_pos[0] + 0.5 * forward_vec[0],
               car_pos[1] + 0.5 * forward_vec[1],
               car_pos[2] + 0.6]
    cam_target = [car_pos[0] + forward_vec[0],
                  car_pos[1] + forward_vec[1],
                  car_pos[2] + 0.3]

    view_matrix = p.computeViewMatrix(cam_pos, cam_target, [0, 0, 1])
    proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.1, farVal=100)

    _, _, rgba_img, _, _ = p.getCameraImage(160, 120, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    img = np.reshape(rgba_img, (120, 160, 4))[:, :, :3]
    return img

def detect_green(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, np.array([35, 40, 40]), np.array([90, 255, 255]))
    green_pixels = np.count_nonzero(mask)
    return green_pixels > 50, mask

def move_husky(car, velocity):
    for joint in [2, 3, 4, 5]:
        p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=velocity, force=1000)

def main():
    car, block_id = setup_environment()

    frame_count = 0

    while True:
        if frame_count % 10 == 0:  # every 10 simulation steps (~0.04s)
            image = get_camera_image(car)
            green_found, mask = detect_green(image)

            #cv2.imshow("Camera View", image)
            #cv2.imshow("Green Mask", mask)
            #cv2.waitKey(1)

            if green_found:
                print("🟢 Green detected! Moving forward")
                move_husky(car, 5)
            else:
                move_husky(car, 0)

        # Check keyboard input
        keys = p.getKeyboardEvents()
        if ord('g') in keys and keys[ord('g')] & p.KEY_WAS_TRIGGERED:
            p.changeVisualShape(block_id, -1, rgbaColor=[0, 1, 0, 1])
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            p.changeVisualShape(block_id, -1, rgbaColor=[1, 0, 0, 1])

        p.stepSimulation()
        time.sleep(1./240.)
        frame_count += 1


if __name__ == "__main__":
    main()
