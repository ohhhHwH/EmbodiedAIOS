# 控制类(虚类) # 仿真控制-->传入的函数替换成仿真器的控制函数
class FuncToolsControl:
    def __init__(self):
        pass
    def get_func_tools_info(self):
        raise NotImplementedError("get_func_tools_info() must be implemented in subclasses")
        pass
    def func_call(self, func_name, arguments):
        raise NotImplementedError("func_call() must be implemented in subclasses")
        pass
    def get_func_tools_list(self):
        raise NotImplementedError("get_func_tools_list() must be implemented in subclasses")
        pass
    
'''
├─ 机械臂控制类 
│ ├─ 机械臂使能
│ ├─ 获取机械臂状态
│ ├─ 机械臂移动
│ ├─ 机械臂急停
│ └─ 机械臂抓取物体
'''
class ArmControl(FuncToolsControl):
    def __init__(self, arm_enable, arm_status, arm_move, arm_stop, arm_grab):
        super().__init__()
        self.arm_enable = arm_enable
        self.arm_status = arm_status
        self.arm_move = arm_move
        self.arm_stop = arm_stop
        self.arm_grab = arm_grab
        '''
        tools_info:一个string,用于描述该类的功能
        该string会被传递给function_call的function参数中的description字段
        '''
        self.tools_info = """
            - arm_move(location: str) # 功能描述: 移动到指定位置 参数描述: 位置(字符串形式)
            - arm_grab(object: str) # 功能描述: 抓取物体 参数描述: 物体名称(字符串形式)
        """
        self.func_tools_list = [
            {# 机械臂移动
                "type": "function",
                "function": {
                    "name": "arm_move",
                    "description": "arm move a location",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "The location the arm to go to",
                            }
                        },
                        "required": ["location"]
                    },
                }
            },
            {# 抓取物体
                "type": "function",
                "function": {
                    "name": "arm_grab",
                    "description": "Grab an object",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "The object to grab",
                            }
                        },
                        "required": ["object"]
                    },
                }
            },
        ]
    # 获取FuncToolsArm类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "arm_enable":
            return self.arm_enable()
        elif func_name == "arm_status":
            return self.arm_status()
        elif func_name == "arm_move":
            return self.arm_move(arguments["location"])
        elif func_name == "arm_grab":
            return self.arm_grab(arguments["object"])
        else:
            raise ValueError(f"Function {func_name} not recognized.")
        
    # 调试接口
    def enable(self):   
        return self.arm_enable()
    def status(self):
        return self.arm_status()
    def move_to(self, location):
        return self.arm_move(location)
    def grab(self, object_name):
        return self.arm_grab(object_name)
    
    

'''
├─ 基座小车控制类 
│ ├─ 小车使能
│ ├─ 获取小车状态
│ ├─ 小车移动
│ ├─ 小车急停
'''
class CarControl(FuncToolsControl):
    def __init__(self, car_enable, car_status, car_move, car_stop):
        super().__init__()
        self.car_enable = car_enable
        self.car_status = car_status
        self.car_move = car_move
        self.car_stop = car_stop
        '''
        tools_info:一个string,用于描述该类的功能
        该string会被传递给function_call的function参数中的description字段
        '''
        self.tools_info = """
            - car_stop() # 功能描述: 停止小车 参数描述: 无
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
            {# 小车急停
                "type": "function",
                "function": {
                    "name": "car_stop",
                    "description": "Stop the car",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        # 无参数
                        # 直接调用函数
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
        if func_name == "car_enable":
            return self.car_enable()
        elif func_name == "car_status":
            return self.car_status()
        elif func_name == "car_move":
            return self.car_move(arguments["location"])
        elif func_name == "car_stop":
            return self.car_stop()
        else:
            raise ValueError(f"Function {func_name} not recognized.")
        
    # 调试接口
    def enable(self):
        return self.car_enable()
    def status(self):
        return self.car_status()
    def move_to(self, location):
        return self.car_move(location)
    def stop(self):
        return self.car_stop()
    





# 函数实现 类初始化 openai_client初始化
'''
├─ 机械臂控制类函数
│ ├─ 获取机械臂状态
│ ├─ 机械臂移动
│ ├─ 机械臂急停
│ └─ 机械臂抓取物体
'''
# 机械臂控制类函数
def arm_enable():
    print("Robot arm enabled")
    return "Robot arm enabled"
def arm_get_status():
    print("Robot arm status: OK")
    return "Robot arm status: OK"
def arm_move(location):
    print(f"Robot arm moving to {location}")
    return f"Robot arm at {location} now"
def arm_stop():
    print("Robot arm emergency stop activated")
    return "Robot arm emergency stop activated"
def arm_grab(object):
    print(f"Robot arm grabbing {object}")
    return f"Robot arm grabbed {object}"
'''
├─ 基座小车控制类 
│ ├─ 小车使能
│ ├─ 获取小车状态
│ ├─ 小车移动
│ ├─ 小车急停
'''
# 小车控制类函数
def car_enable():
    print("Car enabled")
    return "Car enabled"
def car_get_status():
    print("Car status: OK")
    return "Car status: OK"
def car_move(location):
    print(f"Car moving to {location}")
    return f"Car at {location} now"
def car_stop():
    print("Car emergency stop activated")
    return "Car emergency stop activated"   
