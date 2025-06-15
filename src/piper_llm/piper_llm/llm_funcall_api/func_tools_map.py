# -*- coding: utf-8 -*-


import json
import os
import rclpy
from std_srvs.srv import Empty


lang_map = {
        "人": "person",
        "人机": "person",
        "椅子": "chair",
        "桌子": "table",
        "门": "door",
        "汽车": "car",
        "树": "tree",
}

# 地点的语义地图
location_semantic_map = {
    "办公桌": (3.2, 1.6, 0),
    "梅老师办公室": (3.2, 1.6, 0),
    "沙发": (2.5, -1.9, 0),
    "301门口": (-6.3, -4.0, 0),
    "303办公室": (4.0, -4.0, 0),
    "自动售货机": (-11.7, -6.8, 0),
    "厕所": (8.9, -4.8, 0),
    "休息区": (17.0, -6.7, 0),
    "315门口": (29.0, -5.1, 0),
    "曹老师办公室": (28.3, 0.1, 0),
    "办公室": (3.2, -1.3, 0),
    "电梯口": (0.3, -10.9, 0),
    "餐厅": (-8.5, -13.6, 0),
    "饮料": (-11.7, -6.8, 0),
}

map_path = "./map.json"

def load_semantic_map(file_path):
    """
    加载语义地图JSON文件。

    Args:
        file_path (str): 语义地图JSON文件的路径。

    Returns:
        dict: 加载的语义地图数据，如果文件不存在或无效则返回空字典。
    """
    if not os.path.exists(file_path):
        print(f"Error: Semantic map file '{file_path}' not found.")
        return {}
    if os.path.getsize(file_path) == 0:
        print(f"Error: Semantic map file '{file_path}' is empty.")
        return {}
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except json.JSONDecodeError:
        print(f"Error: '{file_path}' is not a valid JSON file.")
        return {}
    except Exception as e:
        print(f"An unexpected error occurred while loading '{file_path}': {e}")
        return {}

def car_get_location(location: str):
    if location in location_semantic_map:
        return location_semantic_map[location]
    else:
        semantic_map_data = load_semantic_map(map_path)
        if location in semantic_map_data:
            x, y = semantic_map_data[location]
            return (x, y, 0)
        else:
            for chinese_object, english_object in lang_map.items():
                if chinese_object == location:
                    if english_object in semantic_map_data:
                        x, y = semantic_map_data[english_object]
                        return (x, y, 0)
    print(f"car_get_location Error '{location}' not found in the semantic map.")
    return None

def map_query(qobject: str):
    quert_str = "map query func: "
    if qobject in location_semantic_map:
        x, y, z = location_semantic_map[qobject]
        quert_str += f"{qobject} at ({x}, {y}, {z})"
    else:
        semantic_map_data = load_semantic_map(map_path)
        if qobject in semantic_map_data:
            x,y = semantic_map_data[qobject]
            quert_str += f"{qobject} at ({x}, {y}, 0)"
        else:
            for chinese_object, english_object in lang_map.items():
                if chinese_object == qobject:
                    if english_object in semantic_map_data:
                        x, y = semantic_map_data[english_object]
                        quert_str += f"{english_object} at ({x}, {y}, 0)"
        print(f"Error '{qobject}' not found in the semantic map.")
    return quert_str

def map_update():
    update_str = "map update func find object: "
    rclpy.init()
    if node is None:
        node = rclpy.create_node('piper_vision_api')
    update_map_client = node.create_client(Empty, "/piper_vision/map_capture")
    while not update_map_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('/piper_vision/map_capture service not available, waiting again...')
    request = Empty.Request()
    future = update_map_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f"Map update successfully")
    node.destroy_node()
    rclpy.shutdown()
    
    semantic_map_data = load_semantic_map(map_path)
    
    for object_name, _ in semantic_map_data.items():
        update_str += f"{object_name}, "
    return update_str

# todo
def get_current_location():
    return 0.0, 0.0, 0.0

def map_update_location(location: str):
    """
    更新地点信息到语义地图。
    
    Args:
        location (str): 地点名称。
        x (float): 地点的x坐标。
        y (float): 地点的y坐标。
        z (float): 地点的z坐标，默认为0。
    
    Returns:
        str: 更新后的地点信息。
    """
    x, y, z = get_current_location()
    location_semantic_map[location] = (x, y, z)
    return f"Updated location '{location}' to coordinates ({x}, {y}, {z})."

    

'''
├─ 语义地图感知

│ ├─ 语义地图更新
│ ├─ 语义地图查询
'''
class MapSensor():
    def __init__(self, map_update=map_update, map_query=map_query):
        super().__init__()
        self.map_update = map_update
        self.map_query = map_query
        
        self.tools_info = """
            - map_update(object: str, info: str) # 功能描述: 更新语义地图 参数描述: 无;返回全部找到的物体
            - map_query(object: str) # 功能描述: 查询语义地图；参数描述: 物体名称；返回物体位置
            - map_record_location(location: str) # 功能描述: 记录当前地点 参数描述: 地点名称和位置；返回更新后的地点信息
        """
        self.func_tools_list = [
            {# 语义地图更新
                "type": "function",
                "function": {
                    "name": "map_update",
                    "description": "Update the semantic map",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    },
                }
            },
            {# 语义地图查询物体
                "type": "function",
                "function": {
                    "name": "map_query",
                    "description": "Query the object/location in the semantic map",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "object_name": {
                                "type": "string",
                                "description": "The object name to query",
                            }
                        },
                        "required": ["object_name"]
                    },
                }
            },
            {# 语义地图更新地点
                "type": "function",
                "function": {
                    "name": "map_update_location",
                    "description": "Update the cur location in the semantic map",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "The location name to update",
                            }
                        },
                        "required": ["location"]
                    },
                }
            },
        ]
    
    # 获取MapSensor类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):

        if func_name == "map_update":
            return self.update_map()
        elif func_name == "map_query":
            return self.map_query(arguments["object"])
        elif func_name == "map_update_location":
            return self.map_query(arguments["location"])
        else:
            raise ValueError(f"Function {func_name} not recognized.")
        
    # 调试接口
    def update_map(self):
        return self.map_update()
    def query_object_map(self, object_name):
        return self.map_query(object_name)
    def query_object_map(self, location_name):
        return self.map_query(location_name)

if __name__ == "__main__":
    # 测试函数
    # map_sensor = MapSensor()
    # print(map_sensor.get_func_tools_info())
    # print(map_sensor.get_func_tools_list())
    # print(map_sensor.update_map())
    # print(map_sensor.query_object_map("沙发"))
    print(car_get_location("沙发")[1])
    
    

