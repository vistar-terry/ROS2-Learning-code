# sensor_processor/sensor_processor/sensor_data_processor.py

class SensorDataProcessor:
    def __init__(self):
        self.reset()

    def reset(self):
        self.latest_data = None

    def process_data(self, data):
        # 在这里添加数据处理的逻辑
        self.latest_data = data * 2  # 示例：将接收到的数据乘以2
        print(f"Processed data: {self.latest_data}")