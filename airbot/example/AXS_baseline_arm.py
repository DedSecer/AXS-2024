# %%
import os
import math
import copy
import numpy as np
import time
import cv2
from threading import Thread, Lock

from airbot.backend import Arm, Camera, Base, Gripper
from airbot.example.utils.draw import draw_bbox, obb2poly
from airbot.backend.utils.utils import camera2base, armbase2world
from airbot.lm.utils import depth2cloud
from scipy.spatial.transform import Rotation

import warnings
warnings.filterwarnings("ignore")


os.environ['LM_CONFIG'] = "/root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/local.yaml"
os.environ['CKPT_DIR'] = '/root/Workspace/AXS_baseline/ckpt'



class Solution:

    CAMERA_SHIFT = np.array([-0.093, 0, 0.07]), np.array([0.5, -0.5, 0.5, -0.5])

    BEFORE_MW_BASE_POSE = (np.array([
        -0.026 + 0.24,
        -0.40,
        -0.17,
    ]), np.array([
        0.00179501,
        0.06667202,
        0.04863613,
        0.99658725,
    ]))

    ARM_POSE_TO_MICROWAVE = (np.array([0.67335, 0.1, -0.05678]), np.array([0.0, 0.0, 0.0, 1.0]))

    POSE_TO_BOWL = (np.array([-0.01226429675289562 + 0.24, 0.11259263609492792, -0.042619463119529605]),
                    np.array([-0.05418989389554988, 0.056031992518506414, 0.707, 0.707]))
    
    ARM_POSE_TO_LOWER_CABINET = (np.array([0.62335, 0.3, -0.05678]), np.array([0.0, 0.0, 0.0, 1.0]))

    ARM_POSE_PUT_LOWER = (np.array([0.62335, 0.3, -0.09678]), np.array([0.0, 0.0, 0.0, 1.0]))

    ARM_POSE_TO_UPPER_CABINET = (np.array([0.62335, 0.3, 0.22]), np.array([0.0, 0.0, 0.0, 1.0]))

    ARM_POSE_PUT_UPPER = (np.array([0.62335, 0.3, 0.17]), np.array([0.0, 0.0, 0.0, 1.0]))

    POSE_OPEN_CAB = (np.array([-0.145 + 0.24, 0.487, -0.235]), np.array([0, 0, 0.750, 0.661]))
    
    ARM_POSE_STANDARD_MOVING = (np.array([0.3225, 0.00, 0.219]), np.array([0.0, 0.0, 0.0, 1.0]))

    GRASP_POSE_1 = (np.array([0.44 + 0.24, 0.71, -0.235]), np.array([0, 0, -0.063, 0.998]))

    GRASP_POSE_2 = (np.array([0.46 + 0.24, 0.697, -0.235]), np.array([0, 0, 0.714, 0.701]))

    B1_OBSERVE_ARM_POSE_1 = (np.array([0.28, 0.2, 0.171663168]), np.array([0, 0.707, 0, 0.707]))
    B1_OBSERVE_ARM_POSE_2 = (np.array([0.28, -0.2, 0.171663168]), np.array([0, 0.707, 0, 0.707]))
    B2_OBSERVE_ARM_POSE = (np.array([0.33, 0.1, 0.171663168]), np.array([0, 0.707, 0, 0.707]))

    END_POSE = (np.array([-1.2317611908186263, -0.9177336410440479, -0.2374505629662929]), 
                np.array([0.006592923324957343, -0.012942241749323641, 0.014944697147015459, 0.9997828203003477]))
    
    bowl_placed = 0
    mug_placed = 0
    has_upper = False
    b1a1 = None
    b1a2 = None
    b2a = None
    color_cab = ['', ''] # color of bowl in the two layer

    def __init__(self, init_base = True, init_arm = True, init_gripper = True, init_camera = True, init_lm = True):
        if init_arm:
            print("arm initial:")
            self.arm = Arm(backend='ros')
            print("arm initial scuceed")
        if init_base:
            print("base initial:")
            self.base = Base(backend='ros')
            print("base initial scuceed")
        if init_gripper:
            print("gripper initial:")
            self.gripper = Gripper(backend='ros')
            print("gripper initial scuceed")
            self.gripper.open()
            print("gripper opened")
        if init_camera:
            print("camera initial:")
            self.camera = Camera(backend='ros')
            print("camera initial scuceed")

        self.image_lock = Lock()
        self.result_lock = Lock()
        self.prompt_lock = Lock()
        self.running = True
        if init_lm:
            self.init_lm()

    def start_lm(self):
        self.t_lm = Thread(target=self.init_lm, daemon=True)
        self.t_lm.start()
    def wait_for_lm(self):
        self.t_lm.join()

    def init_lm(self):
        print("lm initial:")
        from airbot.lm import Detector
        # from airbot.grasp.graspmodel import GraspPredictor
        self.detector = Detector(model='yolo-v8')
        # self.grasper = GraspPredictor(model='graspnet')
        self.prompt = 'cabinet_handle'
        self.update_once()
        self.t_vis = Thread(target=self.vis, daemon=True)
        self.t_vis.start()
        print("lm initial scuceed")

    @property
    def image(self):
        with self.image_lock:
            return copy.deepcopy(self._image)

    @image.setter
    def image(self, value):
        with self.image_lock:
            self._image = copy.deepcopy(value)

    @property
    def prompt(self):
        with self.prompt_lock:
            return copy.deepcopy(self._prompt)

    @prompt.setter
    def prompt(self, value):
        with self.prompt_lock:
            self._prompt = copy.deepcopy(value)

    @property
    def depth(self):
        with self.image_lock:
            return copy.deepcopy(self._depth)

    @depth.setter
    def depth(self, value):
        with self.image_lock:
            self._depth = copy.deepcopy(value)

    @property
    def bbox(self):
        with self.result_lock:
            return copy.deepcopy(self._bbox)

    @bbox.setter
    def bbox(self, value):
        with self.result_lock:
            self._bbox = copy.deepcopy(value)

    def update_once(self):
        self._bbox = None
        with self.image_lock, self.result_lock:
            self._image = copy.deepcopy(self.camera.get_rgb())
            self._depth = copy.deepcopy(self.camera.get_depth())
            self._det_result = self.detector.infer(self._image, self._prompt)
            print(self._det_result)
            self._bbox = self._det_result['bbox'].cpu().numpy().astype(int)
        return self._det_result

    def update_ss(self):
        with self.image_lock, self.result_lock:
            self._image = copy.deepcopy(self.camera.get_rgb())
            self._depth = copy.deepcopy(self.camera.get_depth())
            pose_objs = self.detector.inferss(self._image)
            print(pose_objs)
        return pose_objs


    def update_once_center(self):
        '''get the bbox's score heigher than specify score which closest to the center'''
        self._bbox = None
        with self.image_lock, self.result_lock:
            self._image = copy.deepcopy(self.camera.get_rgb())
            self._depth = copy.deepcopy(self.camera.get_depth())
            self._det_result = self.detector.inferss(self._image, self._prompt, 0.75)
            print(self._det_result)
            min_sum = 10000
            for i in range(len(self._det_result['bbox'])):
                if abs(self._det_result['bbox'][i][1] - 640) + abs(self._det_result['bbox'][i][0] - 360) < min_sum: # closest to the center
                    bbox = self._det_result['bbox'][i]
                    min_sum = abs(self._det_result['bbox'][i][1] - 640) + abs(self._det_result['bbox'][i][0] - 360)
                self._bbox = bbox.numpy().astype(int)

    def update_once_gray(self):
        '''look for gray things'''
        self._bbox = None
        with self.image_lock, self.result_lock:
            self._image = copy.deepcopy(self.camera.get_rgb())
            self._depth = copy.deepcopy(self.camera.get_depth())
            self._det_result = self.detector.inferss(self._image, self._prompt, 0.75)
            print(self._det_result)
        max_score = 0
        for i in range(len(self._det_result['bbox'])):
            bbox = self._det_result['bbox'][i].numpy().astype(int)
            object_rgb = self._image[bbox[0] - np.int32(bbox[2]/4):bbox[0] + np.int32(bbox[2]/4), bbox[1] - np.int32(bbox[3]/4):bbox[1] + np.int32(bbox[3]/4)]
            mean_rgb = (np.mean(np.mean(object_rgb, axis=0), axis=0).astype(int))
            if sum(mean_rgb) > 0 and sum(mean_rgb) < 300 and self._det_result['score'][i] > max_score:
                self._bbox = bbox
                max_score = self._det_result['score'][i]

    def update_once_white(self):
        '''look for white things'''
        self._bbox = None
        with self.image_lock, self.result_lock:
            self._image = copy.deepcopy(self.camera.get_rgb())
            self._depth = copy.deepcopy(self.camera.get_depth())
            self._det_result = self.detector.inferss(self._image, self._prompt, 0.75)
            print(self._det_result)
        max_score = 0
        for i in range(len(self._det_result['bbox'])):
            bbox = self._det_result['bbox'][i].numpy().astype(int)
            object_rgb = self._image[bbox[0] - np.int32(bbox[2]/4):bbox[0] + np.int32(bbox[2]/4), bbox[1] - np.int32(bbox[3]/4):bbox[1] + np.int32(bbox[3]/4)]
            mean_rgb = (np.mean(np.mean(object_rgb, axis=0), axis=0).astype(int))
            if sum(mean_rgb) > 300 and sum(mean_rgb) < 10000 and self._det_result['score'][i] > max_score:
                self._bbox = bbox
                max_score = self._det_result['score'][i]

    def vis(self):
        try:
            # Infinite loop to display images
            while self.running:
                image_draw = self.image
                try:
                    image_draw = draw_bbox(image_draw, obb2poly(self.bbox[None, ...]).astype(int))
                except IndexError:
                    print("bbox index error")
                except TypeError:  # no bbox
                    pass
                image_draw = image_draw.astype(np.uint8)
                image_show = cv2.cvtColor(image_draw, cv2.COLOR_RGB2BGR)
                cv2.putText(image_show,
                            f"prompt: {self._prompt}, det score: {self._det_result['score']}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow('RGB', image_show)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("Exiting due to user interruption.")
        finally:
            cv2.destroyAllWindows()

    @staticmethod
    def _bbox2mask(image, bbox):
        mask = np.zeros_like(image[:, :, 0], dtype=bool)
        mask[
            bbox[0] - bbox[2] // 2:bbox[0] + bbox[2] // 2,
            bbox[1] - bbox[3] // 2:bbox[1] + bbox[3] // 2,
        ] = True
        return mask

    @staticmethod
    def base_cloud(image, depth, intrinsic, shift, end_pose):
        cam_cloud = depth2cloud(depth, intrinsic)
        cam_cloud = np.copy(np.concatenate((cam_cloud, image), axis=2))
        return camera2base(cam_cloud, shift, end_pose)

    def grasp(self):
        if self._bbox is None:
            return False
        with self.image_lock, self.result_lock:
            _depth = copy.deepcopy(self._depth)
            _image = copy.deepcopy(self._image)
            _bbox = copy.deepcopy(self._bbox)

        cloud = self.base_cloud(_image, _depth, self.camera.INTRINSIC, self.CAMERA_SHIFT, self.arm.end_pose)

        grasp_position = cloud[ _bbox[0], _bbox[1] - _bbox[3] // 2 + 8][:3]
        if self._prompt == 'cup':
            grasp_position[2] = -0.168
        else:
            grasp_position[2] = -0.16
        grasp_rotation = Rotation.from_euler('xyz', [0, np.pi / 2, 0], degrees=False).as_quat()

        print('grasp_position',grasp_position)
        print('grasp_rotation',grasp_rotation)
        self.arm.move_end_to_pose(grasp_position, grasp_rotation)

        grasp_position[1] -= 0.04
        grasp_rotation = np.array([-0.13970062182177911, 0.6487791800204252, 0.032918235938941776, 0.75])
        self.arm.move_end_to_pose(grasp_position, grasp_rotation)

        time.sleep(2)
        self.gripper.close()
        time.sleep(6)
        grasp_position[2] = 0
        self.arm.move_end_to_pose(grasp_position, grasp_rotation)
        self.base.backward(3.5)
        grasp_position[1] = 0
        grasp_position[0] = 0.3
        arm_moving_t = Thread(target = self.arm.move_end_to_pose, args = (grasp_position, grasp_rotation))
        arm_moving_t.start() 
        return True

    def grasp_handle(self):
        print("start moving base")
        self.base.move_to(*self.POSE_OPEN_CAB, 'world', False)

        arm_pose = (np.array([0.3225, -0.1, 0.219]), np.array([0, 0, 0, 1]))
        # move_arm_t = Thread(target = self.arm.move_end_to_pose, args = arm_pose)
        self.arm.move_end_to_pose(*arm_pose)

        arm_pose_d = [(np.array([0.3225, 0, 0.219]), np.array([0, 0, 0, 1])),
                    (np.array([0.3225, -0.2, 0.219]), np.array([0, 0, 0, 1])),
                    (np.array([0.3225, -0.1, 0.269]), np.array([0, 0, 0, 1])),
                    (np.array([0.3225, -0.1, 0.169]), np.array([0, 0, 0, 1]))]

        print("start moving arm")
        # move_arm_t.start()

        self.wait_for_lm()
        time.sleep(2)

        print("start to open cabinet")
        self._prompt = 'cabinet_handle'
        res = self.update_once()

        i = 0
        while res['score'] == -1 and i < 4:
            self.arm.move_end_to_pose(*arm_pose_d[i])
            time.sleep(0.5)
            res = self.update_once()
            i += 1

        if i == 4:
            return


        with self.image_lock, self.result_lock:
            _depth = copy.deepcopy(self._depth)
            _image = copy.deepcopy(self._image)
            _bbox = copy.deepcopy(self._bbox)

        cloud = self.base_cloud(_image, _depth, self.camera.INTRINSIC, self.CAMERA_SHIFT, self.arm.end_pose)
        grasp_position = cloud[ _bbox[0], _bbox[1] - _bbox[3] // 2 + 8][:3]
        grasp_position[0] -= 0.01
        # grasp_position[2] = 0.23
        grasp_position[2] = 0.2

        grasp_rotation = np.array([0, 0, 0, 1])
        print('grasp_position',grasp_position)
        print('grasp_rotation', grasp_rotation)
        
        self.arm.move_end_to_pose(grasp_position, grasp_rotation)
        time.sleep(1)
        self.arm.move_end_to_pose(grasp_position, grasp_rotation)
        self.gripper.close()
        time.sleep(5.2)
        r = 0.582
        for i in [0, 1, 1.5]:
            d = 0.1 * (i + 1)
            new_pos = grasp_position + np.array([-r*math.sin(d), r-r*math.cos(d), 0])
            new_ori = Rotation.from_euler("xyz", np.array([0, 0, -d]), degrees=False)
            new_ori = new_ori.as_quat()
            print('i',i)
            self.arm.move_end_to_pose(new_pos, np.array(new_ori))
            time.sleep(0.1)
        self.gripper.open()
        time.sleep(1.3)

        d = 0.35
        new_pos = grasp_position + np.array([-r*math.sin(d), r-r*math.cos(d), 0])
        new_ori = Rotation.from_euler("xyz", np.array([0, 0, -d]), degrees=False)
        new_ori = new_ori.as_quat()
        self.arm.move_end_to_pose(new_pos, np.array(new_ori))
        time.sleep(0.1)
        print(1)
        self.arm.move_end_to_pose(np.array([0.3, 0.00, 0.219]), np.array([0.0, 0.0, 0.0, 1.0]))
        print(2)
        self.arm.move_end_to_pose(np.array([0.3, -0.25, 0.219]), np.array([0.0, 0.0, 0.0, 1.0]))
        print(3)
        self.arm.move_end_to_pose(np.array([0.42, -0.25, 0]), np.array([0.0, 0.0, 0.383, 0.924]))
        print(4)
        self.arm.move_end_to_pose(np.array([0.39, 0, 0]), np.array([0.0, 0.0, 0.383, 0.924]))
        back_t = Thread(target=self.base.backward, args = (2.0, ))
        back_t.start()
        print(5)
        self.arm.move_end_to_pose(np.array([0.39, 0.6, 0]), np.array([0.0, 0.0, 0.383, 0.924]))
   
    def look_for_bowl_in_1(self):
        if self.bowl_placed == 3:
            return
        if self.b1a2 is None:
            arm_move_t = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_2)
        else:
            arm_move_t = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_1)
        arm_move_t.start()
        self.base.move_to(*self.GRASP_POSE_1,'world', False)
        time.sleep(2)
        # look at what objects in the current position
        if self.b1a2 is None:
            print('b1a2')
            self.b1a2 = self.update_ss()
            self.arm.move_end_to_pose(*self.B1_OBSERVE_ARM_POSE_1)
            time.sleep(2)
        if self.b1a1 is None:
            print('b1a1')
            self.b1a1 = self.update_ss()


        for direction in range(2):
            if direction == 0:
                pose_objs = self.b1a1
            elif direction == 1:
                pose_objs = self.b1a2   

            if direction == 0 and len(pose_objs) == 0: # nothing in the direction 0
                self.arm.move_end_to_pose(*self.B1_OBSERVE_ARM_POSE_2)
                time.sleep(2)
                continue
            time.sleep(2)

            for obj_i in range(len(pose_objs)):
                self._prompt = pose_objs[obj_i]
                self.update_once()
                cp = self.lookforonce()
                if cp is None:
                    continue
                centerp_car = np.linalg.inv(np.array(Rotation.from_quat(self.base.rotation).as_matrix())).dot((cp-self.base.position))
                OBSERVE_ARM_POSE_TOP = (np.array([
                            centerp_car[0]- 0.2975 - 0.07,
                            centerp_car[1] + 0.17309,
                            0.018713334665877806,
                    ]), Rotation.from_euler('xyz', [0, np.pi / 2, 0], degrees=False).as_quat())
                self.arm.move_end_to_pose(*OBSERVE_ARM_POSE_TOP)
                time.sleep(2)
                self.update_once()
                self.grasp()
                self.place_bowl()

                if direction == 0 and obj_i+1 != len(pose_objs): # continue to look for in direction 0
                    arm_move_t_ = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_1)
                elif direction == 0 and obj_i+1 == len(pose_objs):  # the last object of direction 0, move to direction 1
                    arm_move_t_ = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_2)
                elif direction == 1:
                    arm_move_t_ = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_2)
                arm_move_t_.start()

                if self.bowl_placed == 3:
                    return

                # skip if nothing in the current position
                if direction == 0 and obj_i+1 == len(pose_objs) and len(self.b1a2) == 0:
                    return
                elif direction == 1 and obj_i+1 == len(pose_objs):
                    return

                self.base.move_to(*self.GRASP_POSE_1,'world', False)
                time.sleep(2)
            
    def look_for_bowl_in_2(self):
        if self.bowl_placed == 3:
            return
        arm_move_t = Thread(target = self.arm.move_end_to_pose, args = self.B2_OBSERVE_ARM_POSE)
        arm_move_t.start()
        self.base.move_to(*self.GRASP_POSE_2,'world', False)
        time.sleep(2)
        # look at what objects in the current position
        if self.b2a is None:
            print('b2a')
            self.b2a = self.update_ss()
        pose_objs = self.b2a

        for obj_i in range(len(pose_objs)):
            self._prompt = pose_objs[obj_i]
            self.update_once()
            cp = self.lookforonce()
            if cp is None:
                continue
            centerp_car = np.linalg.inv(np.array(Rotation.from_quat(self.base.rotation).as_matrix())).dot((cp-self.base.position))
            OBSERVE_ARM_POSE_TOP = (np.array([
                        centerp_car[0]- 0.2975 - 0.07,
                        centerp_car[1] + 0.17309,
                        0.018713334665877806,
                    ]), Rotation.from_euler('xyz', [0, np.pi / 2, 0], degrees=False).as_quat())
            self.arm.move_end_to_pose(*OBSERVE_ARM_POSE_TOP)
            time.sleep(2)
            self.update_once()
            self.grasp()
            self.place_bowl()

            arm_move_t_ = Thread(target = self.arm.move_end_to_pose, args = self.B2_OBSERVE_ARM_POSE)
            arm_move_t_.start()

            if self.bowl_placed == 3:
                return

            # skip if nothing in the current position
            if obj_i+1 == len(pose_objs):
                return

            self.base.move_to(*self.GRASP_POSE_2,'world', False)
            time.sleep(2)

    def place_bowl_to_layer(self, layer):
        if layer == 1:
            self.place_bowl_upper()
        else:
            self.place_bowl_upper()

    def place_bowl(self):
        for layer in range(2):
            if self.color_cab[layer] == self._prompt:
                self.place_bowl_to_layer(layer)
                self.bowl_placed += 1
                print('bowl placed ',self.bowl_placed)
                return
            elif self.color_cab[layer] == '':
                self.place_bowl_to_layer(layer)
                self.color_cab[layer] == self._prompt
                self.bowl_placed += 1
                print('bowl placed ',self.bowl_placed)
                return

        # for third color?
        self.place_bowl_upper()
        self.bowl_placed += 1
        print('bowl placed ',self.bowl_placed)
        return
        

    def look_for_mug_in_1(self):
        self._prompt = 'cup'
        arm_pose_1_t = Thread(target = self.arm.move_end_to_pose, args = self.B1_OBSERVE_ARM_POSE_1)
        arm_pose_1_t.start()
        self.base.move_to(*self.GRASP_POSE_1, 'world', False)
        time.sleep(1)
        cp = None
        for direction in range(2):
            if direction == 0:
                self.b1a1 = self.update_ss()
                try:
                    self.b1a1.remove('cup')
                except ValueError:
                    pass
            elif direction == 1:
                self.arm.move_end_to_pose(*self.B1_OBSERVE_ARM_POSE_2)
                time.sleep(2)
                self.b1a2 = self.update_ss()
                try:
                    self.b1a2.remove('cup')
                except ValueError:
                    pass

            self.update_once()
            cp = self.lookforonce()
            if cp is not None:
                break

        if cp is not None:
            centerp_car = np.linalg.inv(np.array(Rotation.from_quat(self.base.rotation).as_matrix())).dot((cp-self.base.position))
            OBSERVE_ARM_POSE_TOP = (np.array([
                        centerp_car[0]- 0.2975 - 0.05,
                        centerp_car[1] + 0.17309,
                        0.018713334665877806,
                    ]), Rotation.from_euler('xyz', [0, np.pi / 2, 0], degrees=False).as_quat())
            self.arm.move_end_to_pose(*OBSERVE_ARM_POSE_TOP)
            time.sleep(1)
            self.update_once()
            self.grasp()
            self.place_microwave()
            self.mug_placed = True

    def look_for_mug_in_2(self):
        self._prompt = 'cup'
        arm_pose_t = Thread(target = self.arm.move_end_to_pose, args = self.B2_OBSERVE_ARM_POSE)
        arm_pose_t.start()
        self.base.move_to(*self.GRASP_POSE_2, 'world', False)
        time.sleep(2)
        cp = None
        self.b2a = self.update_ss()
        try:
            self.b1a1.remove('cup')
        except ValueError:
            pass

        self.update_once()
        cp = self.lookforonce()

        if cp is not None:
            centerp_car = np.linalg.inv(np.array(Rotation.from_quat(self.base.rotation).as_matrix())).dot((cp-self.base.position))
            OBSERVE_ARM_POSE_TOP = (np.array([
                        centerp_car[0]- 0.2975 - 0.05,
                        centerp_car[1] + 0.17309,
                        0.018713334665877806,
                    ]), Rotation.from_euler('xyz', [0, np.pi / 2, 0], degrees=False).as_quat())
            self.arm.move_end_to_pose(*OBSERVE_ARM_POSE_TOP)
            time.sleep(1)
            self.update_once()
            self.grasp()
            self.place_microwave()
            self.mug_placed = True

    def place_microwave(self):
        self.base.move_to(*self.BEFORE_MW_BASE_POSE, 'world', False)
        self.arm.move_end_to_pose(*self.ARM_POSE_TO_MICROWAVE)
        self.base.forward(7.15)
        self.gripper.open()
        time.sleep(5)
        self.base.backward(3.65)

    def close_microwave(self):
        if not self.mug_placed:
            self.base.move_to(*self.BEFORE_MW_BASE_POSE, 'world', False)
            self.base.forward(3)
        self.arm.move_end_to_pose(*self.ARM_POSE_STANDARD_MOVING)
        self.arm.move_end_to_pose(np.array([0.25, 0.2, 0.05]), np.array([0, 0, 0, 1]))
        self.arm.move_end_to_pose(np.array([0.25, 0.5, 0.05]), np.array([0, 0, 0, 1]))
        self.arm.move_end_to_pose(np.array([0.49, 0.5, 0]), np.array([0, 0, 0, 1]))
        self.arm.move_end_to_pose(np.array([0.5, 0.25, 0]), np.array([0, 0, 0, 1]))
        self.arm.move_end_to_pose(np.array([0.52, -0.1, 0]), np.array([0, 0, 0, 1]))
        self.base.backward(2)

    def place_bowl_lower(self):
        self.base.move_to(*self.POSE_TO_BOWL, 'world', False)
        self.arm.move_end_to_pose(*self.ARM_POSE_TO_LOWER_CABINET)
        input_pose = (np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.base.move_to(*input_pose, 'robot', True)
        self.gripper.open()
        self.arm.move_end_to_pose(*self.ARM_POSE_PUT_LOWER)
        time.sleep(2)
        output_pose = (np.array([- 0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.base.move_to(*output_pose, 'robot', True)
        # self.arm.move_end_to_pose(*self.ARM_POSE_STANDARD_MOVING)

    def place_bowl_upper(self):
        self.base.move_to(*self.POSE_TO_BOWL,'world', False)
        self.arm.move_end_to_pose(*self.ARM_POSE_TO_UPPER_CABINET)
        if self.has_upper: # place bowl more inside if has no bowl on upper
            input_pose = (np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        else:
            input_pose = (np.array([0.7, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.base.move_to(*input_pose, 'robot', True)
        self.gripper.open()
        self.arm.move_end_to_pose(*self.ARM_POSE_PUT_UPPER)
        time.sleep(2)
        output_pose = (np.array([-0.5, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]))
        self.base.move_to(*output_pose, 'robot', True)
        self.has_upper = True
        # self.arm.move_end_to_pose(*self.ARM_POSE_STANDARD_MOVING)

    def lookforonce(self):
        if self.bbox is None or np.all(self.bbox==0):
            return None
        with self.image_lock, self.result_lock:
            _depth = copy.deepcopy(self.camera.get_depth())
            _bbox = self._bbox

        print(f"Found the {self._prompt}")
        centerpoint = depth2cloud(_depth, self.camera.INTRINSIC, organized=True)[_bbox[0] // 1 - 1, _bbox[1] // 1 - 1]
        centerpoint = camera2base(centerpoint, self.CAMERA_SHIFT, self.arm.end_pose)
        centerpoint = (armbase2world(centerpoint, (self.base.position, self.base.rotation)).squeeze())
        print('-' * 50)
        print('centerpoint is', centerpoint)
        print('-' * 50)
        return centerpoint

if __name__ == '__main__':
    s = Solution(init_lm = False, init_base = True)
    print(3)
    s.arm.move_end_to_pose(np.array([0.55, -0.45, 0.05]), np.array([0.0, 0.0, 0.0, 1.0]))