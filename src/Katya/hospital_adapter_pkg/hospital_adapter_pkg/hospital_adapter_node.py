import json
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class HospitalAdapterNode(Node):
    def __init__(self):
        super().__init__('hospital_adapter_node')
        
        
        self.srv = self.create_service(
            Trigger,
            '/get_next_patient_task',
            self.get_next_task_callback
        )
        
        
        user_home = os.path.expanduser('~')
        self.json_path = os.path.join(
            user_home,
            'hospital-robot-ros2',
            'src',
            'Katya',
            'hospital_adapter_pkg',
            'hospital_adapter_pkg',
            'tasks_list.json'
        )
        
        self.get_logger().info(f'Hospital Adapter Node initialized. Reading database from: {self.json_path}')

    def get_next_task_callback(self, request, response):
        """Обработчик вызова сервиса."""
        if not os.path.exists(self.json_path):
            response.success = False
            response.message = f'Error: File {self.json_path} not found.'
            return response

        
        with open(self.json_path, 'r', encoding='utf-8') as f:
            try:
                tasks = json.load(f)
            except json.JSONDecodeError:
                response.success = False
                response.message = 'Error: Invalid JSON format.'
                return response

        
        found_task = None
        for task in tasks:
            if task.get('status') == 'pending':
                found_task = task
                task['status'] = 'in_progress'  # Обновляем статус
                break

        
        if found_task:
            with open(self.json_path, 'w', encoding='utf-8') as f:
                json.dump(tasks, f, indent=2)
            
            patient = found_task.get('patient', 'Unknown')
            room = found_task.get('room', 'Unknown')
            
            response.success = True
            response.message = f'Patient: {patient}, Room: {room}'
            self.get_logger().info(f'Assigned task for {patient} to {room}')
        else:
            response.success = False
            response.message = 'No pending tasks left in database.'
            self.get_logger().warn('No pending tasks available.')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = HospitalAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()