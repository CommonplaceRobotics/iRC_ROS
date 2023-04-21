from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from irc_ros_msgs.msg import CanModuleStates, DioState
from irc_ros_msgs.srv import CanModuleCommand  # , DioCommand
from sensor_msgs.msg import JointState


# For the TUI
from textual.app import App, ComposeResult
from textual.widgets import DataTable


class DataStruct:
    def __init__(self) -> None:
        self.HEADER = (
            "",
            "can_id",
            "pos",
            "set_pos",
            "vel",
            "set_vel",
            "torque",
            "set_torque",
            "DIN",
            "DOUT",
            "ERR",
            "MOTOR",
            "INFO",
        )

        self.internal_data = []  # list of dicts
        self.keys = (
            "module_name",
            "can_id",
            "pos",
            "set_pos",
            "vel",
            "set_vel",
            "torque",
            "set_torque",
            "din",
            "dout",
            "errorstate",
            "motorstate",
        )
        self.rows = []
        self.ii = 0

    def insert_data(self, module_name, data=dict()):
        for entry in self.internal_data:
            if entry["module_name"] == module_name:
                # insert all data where it belongs:
                for key in self.keys:
                    if key in data.keys() and data[key] != "":
                        entry[key] = data[key]
                    elif key not in entry.keys():
                        entry[key] = ""
                return

        # If not already in list append it
        new_entry = {"module_name": module_name}

        # insert all data where it belongs:
        for key in self.keys:
            if key in data.keys():
                new_entry[key] = data[key]
            elif key not in new_entry.keys():
                new_entry[key] = ""

        self.internal_data.append(new_entry)

    def generate_rows(self):
        self.rows.clear()

        for entry in self.internal_data:
            self.rows.append(tuple((entry[key]) for key in self.keys))

        # Sort by the second element in the list, the can id, in ascending order
        self.rows.sort(key=lambda t: t[1])

        self.rows_iter = iter(self.rows)


class TableApp(App):
    def update(self):
        if self.ds is None:
            return

        self.ds.generate_rows()

        # TODO: Find a better way to refresh the cells (Issue #20)
        # This way is losing the selection on each refresh
        self.table.clear()
        self.table.add_rows(self.ds.rows_iter)

    def compose(self) -> ComposeResult:
        yield DataTable()

    def on_mount(self) -> None:
        self.ds = DataStruct()

        self.table = self.query_one(DataTable)

        self.ds.generate_rows()

        self.table.add_columns(*self.ds.HEADER)
        for row in self.ds.rows:
            self.table.add_row(*row, key=row[1])

        self.set_interval(0.1, self.update)

    def on_data_table_cell_selected(self, event) -> None:
        # TODO: Complete this function (Issue #21)

        print(event.cell_key)
        # Get the can_id from the same row, via coordinates?
        # can_id = self.table.at(event.coordinate.x, 1)

        # send command depending on y coordinate
        # Parse table header for this?
        y = event.coordinate.y

        req = CanModuleCommand.Request()
        req.cmd = 0
        if y == 10:
            # Clicked on error state -> reset_error should be called
            req.cmd = 1

        # Dont send the commands until the selection issue with refreshing is not resolved
        # if req.cmd != 0:
        #    dashboard_client.call_async(req)


app = TableApp()


def module_state_callback(data):
    for ms in data.module_states:
        module_state = {}

        module_state["module_name"] = ms.name
        module_state["can_id"] = ms.can_id
        module_state["errorstate"] = ms.error_state
        module_state["motorstate"] = ms.motor_state

        app.ds.insert_data(ms.name, module_state)


def joint_state_callback(data):
    for name, pos, vel, eff in zip(
        data.name, data.position, data.velocity, data.effort
    ):
        module_state = {}

        module_state["module_name"] = name
        module_state["pos"] = pos
        module_state["vel"] = vel
        module_state["torque"] = eff

        app.ds.insert_data(name, module_state)


def dio_state_callback(data):
    print(data)
    if len(data.names) == len(data.inputs):
        out_strs = {}
        for name, value in sorted(zip(data.names, data.inputs)):
            module_name, io_name = name.split("/")

            if module_name not in out_strs.keys():
                out_strs[module_name] = ""

            out_strs[module_name] += str(int(value))

        for key, values in out_strs.items():
            app.ds.insert_data(key, {"din": values})

    elif len(data.names) == len(data.outputs):
        out_strs = {}
        for name, value in sorted(zip(data.names, data.outputs)):
            module_name, io_name = name.split("/")

            if module_name not in out_strs.keys():
                out_strs[module_name] = ""

            out_strs[module_name] += str(int(value))

        for key, values in out_strs.items():
            app.ds.insert_data(key, {"dout": values})


def main():
    rclpy.init()

    # TODO: Dynamic selection of topics, see RTUI how it is done (Issue #57)

    node = rclpy.create_node("irc_ros_cli_dashboard")

    node.create_subscription(
        CanModuleStates, "/dashboard_controller/states", module_state_callback, 10
    )
    node.create_subscription(JointState, "/joint_states", joint_state_callback, 10)
    node.create_subscription(
        DioState, "/dio_controller/get_inputs", dio_state_callback, 10
    )
    node.create_subscription(
        DioState, "/dio_controller/get_outputs", dio_state_callback, 10
    )

    # dashboard_client = node.create_client(
    #     CanModuleCommand, "/dashboard_controller/command"
    # )

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Start CLI loop
    app.run(headless=False)

    ros_thread.join()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
