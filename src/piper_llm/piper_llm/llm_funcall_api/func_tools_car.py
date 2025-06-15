
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # 导入 TaskResult
import sys
from func_tools_map import car_get_location

'''
├─ 基座小车控制类 
│ ├─ 获取小车状态
│ ├─ 小车移动
│ ├─ 小车急停
'''

def setgoal(x, y, yaw):
    rclpy.init()

    navigator = BasicNavigator()

    # 创建目标位姿消息
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg() # 使用当前时间戳
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0 # 通常在2D导航中z为0，但根据你的原始命令保留
    goal_pose.pose.orientation.x = float(yaw)
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # 使用BasicNavigator发送目标位姿
    print('Going to goal pose...')
    navigator.goThroughPoses([goal_pose])

    # 可选：等待导航完成
    # 确保在spin_once循环中处理事件
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # if feedback and feedback.current_pose:
        #     print(f"Current pose: x={feedback.current_pose.pose.position.x:.2f}, "
        #           f"y={feedback.current_pose.pose.position.y:.2f}")
        # 使用 navigator 对象本身进行 spin_once
        rclpy.spin_once(navigator, timeout_sec=5.0)

    rclpy.shutdown()
    
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        return True
    else:
        print('Goal failed!')
        return False

# TODO: 找到小车当前位置
def car_get_status():
    print("Car status: checking...")
    return "Car status: OK"

# 小车移动到指定位置-根据标签移动
def car_move(location):
    move_str = f"car move {location} : "
    location = car_get_location(location)
    if location is None:
        return move_str + "failed - location not found"
    else:
        x, y, z = location[0], location[1], location[2]
        print(f"Moving car to {location} at coordinates ({x}, {y}, {z})")
        # 调用setgoal函数
        ret = setgoal(x, y, z)
        if ret == True:
            return move_str + "success"
        else:
            return move_str + "failed - setgoal error"



'''
├─ 基座小车控制类 
│ ├─ 小车使能
│ ├─ 获取小车状态
│ ├─ 小车移动
│ ├─ 小车急停
'''
class CarControl():
    def __init__(self, car_status=car_get_status, car_move=car_move):
        super().__init__()
        self.car_status = car_status
        self.car_move = car_move

        self.tools_info = """
            - car_move(location: str) # 功能描述: 移动到指定位置 参数描述: 位置(字符串形式)
        """
        self.func_tools_list = [
            {# 小车移动
                "type": "function",
                "function": {
                    "name": "car_move",
                    "description": "car move a location",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "The location the car to go to",
                            }
                        },
                        "required": ["location"]
                    },
                }
            },
        ]
    
    # 获取FuncToolsCar类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "car_status":
            return self.car_status()
        elif func_name == "car_move":
            return self.car_move(arguments["location"])
        else:
            raise ValueError(f"Function {func_name} not recognized.")
        
    # 调试接口
    def status(self):
        return self.car_status()
    def move_to(self, location):
        return self.car_move(location)


if __name__ == "__main__":
    # 测试代码
    car_control = CarControl()
    print(car_control.status())
    print(car_control.move_to("Location A"))

    
    # 测试setgoal函数
    # setgoal(1.0, 2.0, 0.0)  