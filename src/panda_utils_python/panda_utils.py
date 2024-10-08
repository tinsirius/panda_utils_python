import rospy
from actionlib_msgs.msg import GoalStatusArray
from controller_manager_msgs.srv import *
import actionlib
import franka_msgs.msg
import franka_gripper.msg

class panda_utils:
    def __init__(self, ns = ""):
        self.ns = ns

    def recovery_client(self):
        client = actionlib.SimpleActionClient(self.ns + '/franka_control/error_recovery', franka_msgs.msg.ErrorRecoveryAction)
        client.wait_for_server()
        goal = franka_msgs.msg.ErrorRecoveryGoal()
        client.send_goal(goal)
        client.wait_for_result(timeout = rospy.Duration(10))
        return client.get_result()

    def close_gripper(self,width, force, speed, epsilon_inner = 0.1, epsilon_outer = 0.1):
        client = actionlib.SimpleActionClient(self.ns + '/franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        goal.speed = speed
        goal.force = force
        client.send_goal(goal)
        is_success = client.wait_for_result(timeout = rospy.Duration(10))
        if not is_success:
            return False
        result = client.get_result()
        return result.success

    def move_gripper(self,width = 0.07, speed = 0.5):
        client = actionlib.SimpleActionClient(self.ns + '/franka_gripper/move', franka_gripper.msg.MoveAction)
        client.wait_for_server()
        goal = franka_gripper.msg.MoveGoal()
        goal.width = width
        goal.speed = speed
        client.send_goal(goal)
        is_success = client.wait_for_result(timeout = rospy.Duration(10))
        if not is_success:
            return False
        result = client.get_result()
        return result.success

    def stop_gripper(self):
        client = actionlib.SimpleActionClient(self.ns + '/franka_gripper/stop', franka_gripper.msg.StopAction)
        client.wait_for_server()
        goal = franka_gripper.msg.StopGoal()
        client.send_goal(goal)
        is_success = client.wait_for_result(timeout = rospy.Duration(10))
        if not is_success:
            return False
        result = client.get_result()
        return result.success

    def home_gripper(self):
        client = actionlib.SimpleActionClient(self.ns + '/franka_gripper/homing', franka_gripper.msg.HomingAction)
        client.wait_for_server()
        goal = franka_gripper.msg.HomingGoal()
        client.send_goal(goal)
        is_success = client.wait_for_result(timeout = rospy.Duration(10))
        if not is_success:
            return False
        result = client.get_result()
        return result.success
    
    def unload_controller(self, controller_name):
        rospy.wait_for_service(self.ns + '/controller_manager/list_controllers')
        try:
            list_client = rospy.ServiceProxy(self.ns + '/controller_manager/list_controllers', ListControllers)
            list_object = ListControllersRequest()
            controllers = list_client(list_object)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        loaded_controller_name = [controller.name for controller in controllers.controller]
        if controller_name in loaded_controller_name:
            rospy.wait_for_service(self.ns + '/controller_manager/unload_controller')
            try:
                unload_client = rospy.ServiceProxy(self.ns + '/controller_manager/unload_controller', UnloadController)
                unload_object = UnloadControllerRequest()
                unload_object.name = controller_name
                result = unload_client(unload_object)
                if result.ok:
                    print("Unloaded " + controller_name)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return True
        else:
            print("The controller has not been loaded yet")
            return False
        
    def load_controller(self, controller_name):
        rospy.wait_for_service(self.ns + '/controller_manager/list_controllers')
        try:
            list_client = rospy.ServiceProxy(self.ns + '/controller_manager/list_controllers', ListControllers)
            list_object = ListControllersRequest()
            controllers = list_client(list_object)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        loaded_controller_name = [controller.name for controller in controllers.controller]
        if controller_name not in loaded_controller_name:
            rospy.wait_for_service(self.ns + '/controller_manager/load_controller')
            try:
                load_client = rospy.ServiceProxy(self.ns + '/controller_manager/load_controller', LoadController)
                load_object = LoadControllerRequest()
                load_object.name = controller_name
                result = load_client(load_object)
                if result.ok:
                    print("Loaded " + controller_name)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                return True
        else:
            print("The controller is already loaded")
            return False

    def switch_controller(self, controller_names):
        if not isinstance(controller_names, list):
            print("controller_names has to be list")
            return False
        rospy.wait_for_service(self.ns + '/controller_manager/list_controllers')
        try:
            list_client = rospy.ServiceProxy(self.ns + '/controller_manager/list_controllers', ListControllers)
            list_object = ListControllersRequest()
            controllers = list_client(list_object)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        is_loaded = False
        for controller in controllers.controller:
            if controller.name in controller_names:
                is_loaded = True
        
        if is_loaded:
            active_controllers = []
            for controller in controllers.controller:
                if controller.state == 'running':
                    # I mean I can use any() but there are only 2 so this is more readable, fight me!
                    if ('state' not in controller.name) and ('gripper' not in controller.name):
                        active_controllers.append(controller.name)
            for active_controller in active_controllers:
                if active_controller in controller_names:
                    print("robot is already running " + active_controller)
                    return True
            else:
                rospy.wait_for_service(self.ns + '/controller_manager/switch_controller')
                try:
                    switch_client = rospy.ServiceProxy(self.ns + '/controller_manager/switch_controller', SwitchController)
                    switch_object = SwitchControllerRequest()
                    switch_object.stop_controllers = active_controllers if active_controllers else ['']
                    switch_object.start_controllers = controller_names
                    switch_object.strictness = 2
                    switch_object.start_asap = False
                    switch_object.timeout = 0.0
                    result = switch_client(switch_object)
                    if result.ok:
                        print("Switch to " + str(controller_names))
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                return True
        else:
            print("Controller is not loaded yet")
            return False
