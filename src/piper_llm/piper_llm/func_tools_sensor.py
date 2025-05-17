

# 感知类(虚类)
class FuncToolsSensor:
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
├─ 视觉感知类
│ ├─ 相机开启
│ ├─ 图像获取
│ ├─ 图像中物体检测
│ └─ 目标检测与定位
'''
class VisionSensor(FuncToolsSensor):
    def __init__(self, vision_camera_enable, vision_image_get, vision_object_find, vision_object_detection):
        super().__init__()
        self.vision_camera_enable = vision_camera_enable
        self.vision_image_get = vision_image_get
        self.vision_object_find = vision_object_find
        self.vision_object_detection = vision_object_detection
        
        self.current_image = None

        self.tools_info = """
            - vision_image_get() # 功能描述: 图像获取 参数描述: 无
            - vision_object_find(object: str) # 功能描述: 找到物体 参数描述: 物体名称(字符串形式)
            - vision_object_detection() # 功能描述: 物体标记 参数描述: 无
        """
        self.func_tools_list = [
            {# 图像获取
                "type": "function",
                "function": {
                    "name": "image_get",
                    "description": "Get image data",
                }
            },
            {# 图像中找到物体
                "type": "function",
                "function": {
                    "name": "find_object",
                    "description": "Find an object in the image",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "The object to find",
                            }
                        },
                        "required": ["object"]
                    },
                }
            },
            {# 物体标记
                "type": "function",
                "function": {
                    "name": "object_detection",
                    "description": "Find an object in the image",
                }
            }
        ]
    # 获取VisionSensor类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "vision_camera_enable":
            return self.camera_open()
        elif func_name == "vision_image_get":
            return self.image_get()
        elif func_name == "vision_object_find":
            return self.find_object(arguments["object"])
        elif func_name == "vision_object_detection":
            return self.detect_object()
        else:
            raise ValueError(f"Function {func_name} not recognized.")
    
    # 调试接口
    def camera_open(self):
        return self.vision_camera_enable()
    def image_get(self):
        return self.vision_image_get()
    def find_object(self, object_name, image_data=None):
        return self.vision_object_find(object_name, image_data)
    def detect_object(self):
        return self.vision_object_detection()


'''
├─ 听觉感知类
│ ├─ 音频采集
│ ├─ 语音转文本
│ └─ 文本转语音
'''
class AudioSensor(FuncToolsSensor):
    def __init__(self, audio_capture, audio_speech_to_text, audio_text_to_speech):
        super().__init__()
        self.audio_capture = audio_capture
        self.speech_to_text = audio_speech_to_text
        self.text_to_speech = audio_text_to_speech
        

        self.tools_info = """
            - audio_capture(time: int) # 功能描述: 音频采集 参数描述: 时间(整数形式)
            - audio_speech_to_text(at: str) # 功能描述: 语音转文本 参数描述: 音频位置(字符串形式)
            - audio_text_to_speech(text: str) # 功能描述: 文本转语音 参数描述: 文本数据(字符串形式)
        """
        self.func_tools_list = [
            {
                "type": "function",
                "function": {
                    "name": "audio_capture",
                    "description": "Capture audio",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "audio_data": {
                                "type": "string",
                                "description": "The audio data to capture",
                            }
                        },
                        "required": ["audio_data"]
                    },
                }
            },
        ]

    # 获取AudioSensor类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "audio_capture":
            return self.capture_audio(arguments["time"])
        elif func_name == "audio_speech_to_text":
            return self.speech_to_text(arguments["audio_data"])
        elif func_name == "audio_text_to_speech":
            return self.text_to_speech(arguments["text"])
        else:
            raise ValueError(f"Function {func_name} not recognized.")

    # 调试接口
    def capture_audio(self, time):
        return self.audio_capture(time)
    def speech_to_text(self, audio_data):
        return self.audio_speech_to_text(audio_data)
    def text_to_speech(self, text):
        return self.audio_text_to_speech(text)


'''
map中的功能嵌入到FuncToolsVision类中?
├─ 语义地图感知
│ ├─ 语义地图创建
│ ├─ 语义地图更新
│ ├─ 语义地图查询
│ └─ 语义地图可视化
'''
class MapSensor(FuncToolsSensor):
    def __init__(self, map_create, map_update, map_query_class, map_query_object, map_visualize=None):
        super().__init__()
        self.map_create = map_create
        self.map_update = map_update
        self.map_query_class = map_query_class
        self.map_query_object = map_query_object
        self.map_visualize = map_visualize
        

        # self.tools_info = """
        #     - map_create() # 功能描述: 创建语义地图 参数描述: 无
        #     - map_update(object: str, info: str) # 功能描述: 更新语义地图 参数描述: 无
        #     - map_query_class() # 功能描述: 查询语义地图 参数描述: 物体类别
        #     - map_query_object(object: str) # 功能描述: 查询语义地图 参数描述: 物体名称
        #     - map_visualize() # 功能描述: 可视化语义地图 参数描述: 无
        # """
        
        self.tools_info = """
            - map_query_class(class: str) # 功能描述: 查询语义地图；参数描述: 物体类别；返回类别的物体
            - map_query_object(object: str) # 功能描述: 查询语义地图；参数描述: 物体名称；返回物体位置
        """
        self.func_tools_list = [
            {# 语义地图查询 
                "type": "function",
                "function": {
                    "name": "query_class",
                    "description": "Query the semantic map",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "object_class": {
                                "type": "string",
                                "description": "The object class to query",
                            }
                        },
                        "required": ["object_class"]
                    },
                }
            },
            {# 语义地图查询
                "type": "function",
                "function": {
                    "name": "query_object",
                    "description": "Query the semantic map",
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
            
        ]
    
    # 获取MapSensor类的tools信息 提供给function_call使用
    def get_func_tools_info(self):
        return self.tools_info
    def get_func_tools_list(self):
        return self.func_tools_list
    
    # 调用func_call函数来执行相应的功能
    def func_call(self, func_name, arguments):
        if func_name == "map_create":
            return self.create_map()
        elif func_name == "map_update":
            return self.update_map()
        elif func_name == "map_query_class":
            return self.map_query_class(arguments["class"])
        elif func_name == "map_query_object":
            return self.map_query_object(arguments["object"])
        elif func_name == "map_visualize":
            return self.visualize_map()
        else:
            raise ValueError(f"Function {func_name} not recognized.")
        
    # 调试接口
    def create_map(self):
        return self.map_create()
    def update_map(self):
        return self.map_update()
    def query_class_map(self, class_name):
        return self.map_query_class(class_name)
    def query_object_map(self, object_name):
        return self.map_query_object(object_name)
    def visualize_map(self):
        return self.map_visualize()



'''
├─ 雷达感知(TODO)
│ ├─ 雷达使能
...


'''


'''
├─ 视觉感知类
│ ├─ 相机开启
│ ├─ 图像获取
│ ├─ 图像中物体检测
│ └─ 目标检测与定位
'''
# 函数名称:camera_enable()
# 函数描述:相机开启
# 输入:none
# 输出:none
def camera_enable():
    print("Camera enabled")
    return "Camera enabled"
# 函数名称:image_capture()
# 函数描述:图像获取
# 输入:none
# 输出:str(图片数据的位置)
def image_capture():
    print("Image captured")
    return "Image captured at ..."
# 函数名称:object_find()
# 函数描述:图像中物体检测
# 输入:str(要查找的物体名称), option:str(图片数据存储的位置)
# 输出:str(物体位置信息-标签/坐标)
# eg1:"杯子 在 x,y,z"
# eg1:"杯子 在 标签1处", 标签1需要对应一个坐标能够使其他函数能够获取使用
def object_find(target, image=None):
    print(f"Target location for {target} in {image}")
    return f"{target} located at x, y, z"
# 函数名称:object_detection()
# 函数描述:将图像中的物体进行标记-加入到语义地图?
# 输入:option:str(图片数据的位置)
# 输出:str(当前场景中的物体)
def object_detection(image=None):
    print(f"Object detection on {image}")
    return "Object detected"



'''
├─ 听觉感知类
│ ├─ 音频采集
│ ├─ 语音转文本
│ └─ 文本转语音
'''
# 函数名称:audio_capture()
# 函数描述:音频采集
# 输入:int(采集时间)
# 输出:str(音频数据的位置)
def audio_capture(time=10):
    print("Audio captured")
    return f"Audio captured {time} at ..."

# 函数名称:speech_to_text()
# 函数描述:语音转文本
# 输入:str(音频数据的位置)
# 输出:str(文本数据)
def speech_to_text(audio):
    print(f"Converting {audio} to text")
    return "Text from audio"
# 函数名称:text_to_speech()
# 函数描述:文本转语音
# 输入:str(文本数据)
# 输出:none;扬音器播放语音
def text_to_speech(text):
    print(f"Converting {text} to speech")
    return "Speech from text"


'''
├─ 语义地图感知
│ ├─ 语义地图创建
│ ├─ 语义地图更新
│ ├─ 语义地图查询
'''
# 函数名称:semantic_map_create()
# 函数描述:语义地图创建
def semantic_map_create():
    print("Creating semantic map")
    return "Semantic map created"
# 函数名称:semantic_map_update()
# 语义地图更新-图像感知时调用该函数更新语义地图?
# 参数待定
def semantic_map_update():
    print("Updating semantic map")
    return "Semantic map updated"
# 函数名称:semantic_map_query
# 函数描述:语义地图中查询物体/类别
# 输入:str(要查询的物体/类别)
# 输出:str(物体位置信息/类别的种类描述)
def semantic_map_query(object):
    print(f"Querying {object} semantic map")
    return f"{object} found at x, y, z"



