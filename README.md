# Data Bridge Package

High-performance C++ WebSocket bridge for real-time data streaming between ROS 2 and web clients.

## Overview

The data_bridge package provides an efficient Protocol Buffer-based WebSocket server that streams ROS 2 data to web applications. It replaces traditional JSON-based solutions (like rosbridge) with a high-performance binary protocol.

## Features

- **High Performance**: C++ implementation with binary Protocol Buffer encoding
- **Real-time Streaming**: WebSocket server for low-latency communication
- **Selective Topics**: Configurable topic filtering for CAN data, diagnostics, and robot state
- **Auto-reconnection**: Robust client connection management
- **Thread-safe**: Multi-client support with proper synchronization

## Architecture

```
ROS 2 Topics → proto_bridge → WebSocket (Port 9090) → Web Clients
```

### Supported Topics

- `/can/raw` - Raw CAN frames
- `/can/tx` - Transmitted CAN messages  
- `/can/rx` - Received CAN messages
- `/diagnostics` - System diagnostic information
- `/robot_state` - Robot status messages

## Building

```bash
# Install dependencies (if not already done)
sudo apt install libwebsocketpp-dev nlohmann-json3-dev

# Build the package
colcon build --packages-select data_bridge
```

## Usage

### Direct Execution
```bash
# Source workspace
source install/setup.bash

# Start the bridge
ros2 run data_bridge proto_bridge
```

### Using Launch File
```bash
# Start with custom port
ros2 launch data_bridge proto_bridge.launch.py port:=9090
```

### Using Robot Control Script
```bash
# Start bridge only
./scripts/robot.sh bridge

# Start complete system (includes bridge)
./scripts/robot.sh full
```

## Configuration

The bridge connects to WebSocket clients on port 9090 by default. Clients receive real-time Protocol Buffer encoded messages.

### Protocol Buffer Schema

See `proto/evabot.proto` for the complete message definitions:

```protobuf
message CanFrame {
  uint32 id = 1;
  bytes data = 2;
  uint64 timestamp = 3;
}

message RobotState {
  string state = 1;
  uint64 timestamp = 2;
}
```

## Client Connection

Web clients can connect via WebSocket:

```javascript
const ws = new WebSocket('ws://localhost:9090');
ws.binaryType = 'arraybuffer';

ws.onmessage = (event) => {
  // Decode Protocol Buffer message
  const message = EvaBot.Message.decode(new Uint8Array(event.data));
  console.log('Received:', message);
};
```

## Performance

- **Latency**: <1ms for local connections
- **Throughput**: >1000 messages/second
- **Memory**: Efficient binary encoding vs JSON
- **CPU**: Optimized C++ implementation

## Troubleshooting

### Build Issues
```bash
# Check dependencies
pkg-config --exists websocketpp
find /usr/include -name "json.hpp"

# Clean rebuild
rm -rf build/data_bridge install/data_bridge
colcon build --packages-select data_bridge
```

### Runtime Issues
```bash
# Check if bridge is running
ros2 node list | grep proto_bridge

# Test WebSocket connection
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Key: test" -H "Sec-WebSocket-Version: 13" \
  http://localhost:9090/
```

### Topic Issues
```bash
# List available topics
ros2 topic list

# Check topic data
ros2 topic echo /can/raw
```

## Development

### Adding New Topics

1. Update topic subscriptions in `src/proto_bridge.cpp`
2. Add message types to `proto/evabot.proto`
3. Rebuild package

### Testing

```bash
# Start bridge
ros2 run data_bridge proto_bridge

# In another terminal, publish test data
ros2 topic pub /can/raw can_msgs/msg/Frame "{id: 123, data: [1,2,3,4]}"
```

## Dependencies

- **ROS 2 Humble** - Core framework
- **WebSocket++** - WebSocket server implementation
- **nlohmann/json** - JSON handling (for configuration)
- **Protocol Buffers** - Message serialization

## See Also

- [CAN Gateway Package](../can_gateway/README.md) - CAN bus interface
- [Evabot Bringup](../evabot_bringup/README.md) - System launch files
- [Web Interface](../../web_interface/README.md) - Frontend application

## Cross-references

- For workspace setup, see [../../README.md](../../README.md).
- For system-wide launch files, see [evabot_bringup/README.md](../evabot_bringup/README.md).
