import csv
from datetime import datetime
from pathlib import Path
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory
from message_filters import Subscriber, ApproximateTimeSynchronizer

import yaml

from robot_simulation.metrics import (
    mae,
    rmse,
    max_absolute_error,
    final_error,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp,
    )


def angle_difference(reference, estimate):
    difference = reference - estimate

    return math.atan2(
        math.sin(difference),
        math.cos(difference),
    )


class StateComparator(Node):

    def __init__(self):
        super().__init__('state_comparator')

        # --------------------------------------------------
        # Load configuration
        # --------------------------------------------------

        config_path = (
            Path(
                get_package_share_directory(
                    'robot_simulation'
                )
            )
            / 'config'
            / 'comparison'
            / 'baseline.yaml'
        )

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        comparison = config['comparison']

        self.comparison_name = comparison['name']

        self.reference_name = (
            comparison['reference']['name']
        )

        self.reference_topic = (
            comparison['reference']['topic']
        )

        self.estimate_name = (
            comparison['estimate']['name']
        )

        self.estimate_topic = (
            comparison['estimate']['topic']
        )

        self.variables = comparison['variables']

        sync_config = comparison['synchronization']

        self.queue_size = sync_config['queue_size']
        self.tolerance_sec = sync_config['tolerance_sec']

        # --------------------------------------------------
        # Comparison state
        # --------------------------------------------------

        self.active = False

        self.errors = {
            variable: []
            for variable in self.variables
        }

        self.sample_count = 0
        self.samples = []

        self.output_dir = Path.home() / 'ros2_ws' / 'results'
        self.output_dir.mkdir(
            parents=True,
            exist_ok=True,
        )        

        # --------------------------------------------------
        # Experiment lifecycle subscription
        # --------------------------------------------------

        self.create_subscription(
            Bool,
            '/experiment/active',
            self.active_callback,
            10,
        )

        # --------------------------------------------------
        # Time-synchronized subscriptions
        # --------------------------------------------------

        self.reference_subscriber = Subscriber(
            self,
            Odometry,
            self.reference_topic,
        )

        self.estimate_subscriber = Subscriber(
            self,
            Odometry,
            self.estimate_topic,
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [
                self.reference_subscriber,
                self.estimate_subscriber,
            ],
            queue_size=self.queue_size,
            slop=self.tolerance_sec,
        )

        self.synchronizer.registerCallback(
            self.synced_callback
        )

        self.get_logger().info(
            f'Comparator ready: {self.comparison_name}'
        )

        self.get_logger().info(
            'Waiting for experiment start...'
        )

    # --------------------------------------------------
    # Experiment lifecycle
    # --------------------------------------------------

    def active_callback(self, msg):

        # Start experiment
        if msg.data and not self.active:

            self.reset()
            

            self.active = True

            self.get_logger().info(
                'Comparison started'
            )

            return

        # Stop experiment

            
        if not msg.data and self.active:

            self.active = False

            self.get_logger().info(
                'Comparison stopped'
            )

            self.print_report()
            self.save_csv()            
            

    def reset(self):

        self.errors = {
            variable: []
            for variable in self.variables
        }

        self.sample_count = 0
        self.samples = []

    # --------------------------------------------------
    # State extraction
    # --------------------------------------------------

    def extract_state(self, msg):

        pose = msg.pose.pose
        twist = msg.twist.twist

        return {
            'x': pose.position.x,
            'y': pose.position.y,

            'yaw': yaw_from_quaternion(
                pose.orientation
            ),

            'linear_velocity': (
                twist.linear.x
            ),

            'angular_velocity': (
                twist.angular.z
            ),
        }

    # --------------------------------------------------
    # Synchronized comparison
    # --------------------------------------------------

    def synced_callback(
        self,
        reference_msg,
        estimate_msg,
    ):

        if not self.active:
            return

        reference_state = self.extract_state(
            reference_msg
        )

        estimate_state = self.extract_state(
            estimate_msg
        )

        for variable in self.variables:

            reference_value = (
                reference_state[variable]
            )

            estimate_value = (
                estimate_state[variable]
            )

            if variable == 'yaw':

                error = angle_difference(
                    reference_value,
                    estimate_value,
                )

            else:

                error = (
                    reference_value -
                    estimate_value
                )

            self.errors[variable].append(
                error
            )

        self.sample_count += 1


        timestamp = (
            reference_msg.header.stamp.sec
            + reference_msg.header.stamp.nanosec * 1e-9
        )

        sample = {
            'time': timestamp,
        }

        for variable in self.variables:

            sample[f'reference_{variable}'] = (
                reference_state[variable]
            )

            sample[f'estimate_{variable}'] = (
                estimate_state[variable]
            )

            sample[f'error_{variable}'] = (
                self.errors[variable][-1]
            )

        self.samples.append(sample)        
        

    # --------------------------------------------------
    # Report
    # --------------------------------------------------
    def print_report(self):

        lines = []

        lines.append('')
        lines.append('========================================')
        lines.append('STATE COMPARISON REPORT')
        lines.append('========================================')

        lines.append(
            f'Comparison: {self.comparison_name}'
        )

        lines.append(
            f'Reference: {self.reference_name}'
        )

        lines.append(
            f'Estimate: {self.estimate_name}'
        )

        lines.append(
            f'Synchronized samples: {self.sample_count}'
        )

        lines.append('----------------------------------------')

        for variable in self.variables:

            errors = self.errors[variable]

            if not errors:
                lines.append(
                    f'{variable}: no synchronized samples'
                )
                continue

            lines.append('')
            lines.append(variable)

            lines.append(
                f'  RMSE: {rmse(errors):.6f}'
            )

            lines.append(
                f'  MAE: {mae(errors):.6f}'
            )

            lines.append(
                f'  Max abs error: '
                f'{max_absolute_error(errors):.6f}'
            )

            lines.append(
                f'  Final error: '
                f'{final_error(errors):.6f}'
            )

        lines.append('')
        lines.append('========================================')

        report = '\n'.join(lines)

        self.get_logger().info(report)
        
    def save_csv(self):

        if not self.samples:
            self.get_logger().warning(
                'No samples available for CSV export'
            )
            return

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        filename = (
            f'{self.comparison_name}_{timestamp}.csv'
        )

        output_path = (
            self.output_dir / filename
        )

        fieldnames = list(
            self.samples[0].keys()
        )

        with open(
            output_path,
            'w',
            newline='',
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames,
            )

            writer.writeheader()
            writer.writerows(
                self.samples
            )

        self.get_logger().info(
            f'Results saved to: {output_path}'
        )        

    

def main(args=None):

    rclpy.init(args=args)

    node = StateComparator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()