ros2 service call /node_name/set_parameters rcl_interfaces/srv/SetParameters "{parameters: [{name: "bool_param", value: {type: 1, bool_value: False}}, {name: "string_param", value: {type: 4, string_value: "test_string"}}, {name: "double_param", value: {type: 3, double_value: 0.01}}, {name: "vector_param", value: {type: 8, double_array_value: [0.1, 0.2, 0.3]}}, {name: "int_param", value: {type: 2, integer_value: 3000}}]}"