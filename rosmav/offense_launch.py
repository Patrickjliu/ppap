launch:

- node:
    pkg: "robot"
    exec: "test_node"
    name: "test_node"
    namespace: "test"

- node:
    pkg: "robot"
    exec: "heading_control"
    name: "heading"
    namespace: "heading"

- node:
    pkg: "robot"
    exec: "defensive_depth_control"
    name: "depth"
    namespace: "depth"

- node:
    pkg: "robot"
    exec: "april_control"
    name: "april_control"
    namespace: "april"
