# ORIGINAL SCRIPT — "Autonomous Obstacle Avoidance with Vision"
# DYSL-AT, DRDO internship report (Section 3.8), kept unchanged for reference.
# The integrated system re-uses get_depth_image_reliable() and
# detect_obstacle_regions() from here via drone/avoidance.py, and turns the
# blocking while-loop below into a non-blocking controller (drone/controller.py)
# so the GARUDA OPS dashboard can command the drone and stream telemetry.
import airsim
import numpy as np
import cv2
import time


def get_depth_image_reliable(client, camera_name="0"):
    responses = client.simGetImages([airsim.ImageRequest(camera_name,
                                     airsim.ImageType.DepthPerspective, True, False)])
    if not responses or responses[0].image_data_float is None:
        return None

    depth_array = np.array(responses[0].image_data_float, dtype=np.float32)
    height = responses[0].height
    width = responses[0].width
    if height == 0 or width == 0:
        return None
    depth_img = depth_array.reshape(height, width)
    depth_img[depth_img == 0] = np.inf

    return depth_img


def detect_obstacle_regions(depth_img, threshold=3.0, roi_height_ratio=0.6):
    h, w = depth_img.shape
    roi_start_row = int(h * (1.0 - roi_height_ratio))
    roi_depth_img = depth_img[roi_start_row:, :]

    third = w // 3
    left_region = roi_depth_img[:, :third]
    center_region = roi_depth_img[:, third:2*third]
    right_region = roi_depth_img[:, 2*third:]

    def min_dist(region):
        if region.size == 0 or np.all(np.isinf(region)):
            return np.inf
        return np.min(region)

    return {
        'left': min_dist(left_region) < threshold,
        'center': min_dist(center_region) < threshold,
        'right': min_dist(right_region) < threshold,
        'min_dist_left': min_dist(left_region),
        'min_dist_center': min_dist(center_region),
        'min_dist_right': min_dist(right_region)
    }


def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(-5, 1).join()

    forward_speed = 1.0
    avoid_speed_sideways = 1.0
    avoid_duration = 1.5
    look_ahead_threshold = 5.0

    try:
        while True:
            depth_img = get_depth_image_reliable(client)
            if depth_img is None:
                time.sleep(0.1)
                continue

            obstacles_info = detect_obstacle_regions(depth_img,
                                                     threshold=look_ahead_threshold, roi_height_ratio=0.6)

            if obstacles_info['center']:
                vx, vy, yaw_rate = 0, 0, 0
                if not obstacles_info['left']:
                    vy = -avoid_speed_sideways
                elif not obstacles_info['right']:
                    vy = avoid_speed_sideways
                else:
                    yaw_rate = 30

                client.moveByVelocityAsync(vx, vy, 0, avoid_duration,
                                           drivetrain=airsim.DrivetrainType.ForwardOnly,
                                           yaw_mode=airsim.YawMode(True, yaw_rate)).join()
                time.sleep(1.0)
                if yaw_rate != 0:
                    client.moveByVelocityAsync(0, 0, 0, 0.1,
                                               yaw_mode=airsim.YawMode(True, 0)).join()
            else:
                vx = forward_speed
                vy = 0
                if obstacles_info['left'] and not obstacles_info['right']:
                    vy = 0.5
                elif obstacles_info['right'] and not obstacles_info['left']:
                    vy = -0.5

                client.moveByVelocityAsync(vx, vy, 0, 0.5,
                                           drivetrain=airsim.DrivetrainType.ForwardOnly,
                                           yaw_mode=airsim.YawMode(False, 0))
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
