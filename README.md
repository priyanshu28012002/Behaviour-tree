# Behavior Trees

## Overview
Behavior Trees (BTs) are a powerful tool for modeling complex decision-making systems in robotics and AI. This document explains the core concepts and components of Behavior Trees in a general context.

## Node Types

### 1. Control Flow Nodes

#### Sequence (→)
- Executes children in order
- Returns `SUCCESS` if all children succeed
- Returns `FAILURE` if any child fails
- Returns `RUNNING` if any child is running

```xml
<Sequence>
  <OpenDoor/>
  <EnterRoom/>
  <CloseDoor/>
</Sequence>
```

#### Fallback (Selector) (?)
- Tries children in order until one succeeds
- Returns `SUCCESS` if any child succeeds
- Returns `FAILURE` if all children fail
- Returns `RUNNING` if any child is running

```xml
<Fallback>
  <OpenDoor/>
  <BreakWindow/>
  <CallForHelp/>
</Fallback>
```

#### Parallel (⇉)
- Executes all children concurrently
- Returns based on threshold policy:
  - `SUCCESS` if at least N children succeed
  - `FAILURE` if at least M children fail
  - `RUNNING` otherwise

```xml
<Parallel success_threshold="2" failure_threshold="2">
  <MonitorBattery/>
  <MonitorTemperature/>
  <MonitorNetwork/>
</Parallel>
```

### 2. Execution Nodes

#### Action Nodes
- Perform actual tasks
- Can return `SUCCESS`, `FAILURE`, or `RUNNING`
- Blocking or non-blocking implementations

```xml
<MoveTo goal="{target}"/>
<Grasp object="cup"/>
```

#### Condition Nodes
- Check state without modifying it
- Return only `SUCCESS` or `FAILURE`
- Should execute quickly

```xml
<IsDoorOpen/>
<BatteryLevelAbove threshold="30"/>
```

### 3. Decorator Nodes
Modify behavior of a single child node

#### Inverter (¬)
- Inverts child's result (`SUCCESS` ↔ `FAILURE`)

```xml
<Inverter>
  <IsDoorLocked/>
</Inverter>
```

#### Retry (↺)
- Repeats child until it succeeds or max attempts

```xml
<Retry num_attempts="3">
  <OpenDoor/>
</Retry>
```

#### Repeat (↻)
- Repeats child N times or indefinitely

```xml
<Repeat num_cycles="5">
  <ScanArea/>
</Repeat>
```

#### Timeout (⌛)
- Fails if child doesn't complete in time

```xml
<Timeout msec="5000">
  <EstablishConnection/>
</Timeout>
```

## Blackboard
Shared memory space for nodes to exchange data

### Usage Patterns
```cpp
// Writing to blackboard
blackboard->set("target_pose", pose);

// Reading from blackboard
auto pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
```

## Tick Semantics
- Trees are "ticked" from root at regular intervals
- Nodes return:
  - `SUCCESS`: Task completed successfully
  - `FAILURE`: Task cannot be completed
  - `RUNNING`: Task still executing
  - `IDLE`: Not yet executed

## Best Practices

### Design Principles
1. Keep trees shallow and wide rather than deep
2. Use subtrees for complex behaviors
3. Favor reactive behaviors over complex planning
4. Separate conditions from actions

### Common Patterns

#### Reactive Monitoring
```xml
<ReactiveSequence>
  <IsBatteryLow/>
  <ReturnToCharger/>
</ReactiveSequence>
```

#### Error Recovery
```xml
<Fallback>
  <MainApproach/>
  <Sequence>
    <Inverter>
      <IsApproachSafe/>
    </Inverter>
    <RecoveryBehavior/>
  </Sequence>
</Fallback>
```

## Implementation Notes

### Synchronous vs Asynchronous
- **Synchronous**: All nodes ticked in one cycle
- **Asynchronous**: Nodes can run across multiple ticks

### Thread Safety
- Blackboard access should be thread-safe
- Consider mutexes for shared resources
- Avoid blocking operations in conditions

## Tools & Libraries
- BehaviorTree.CPP (C++)
- py_trees (Python)
- ROS 2 Behavior Trees
- Unreal Engine Behavior Tree Editor

## Example Tree
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence>
      <IsSystemOk/>
      <Sequence>
        <SelectTarget/>
        <MoveToGoal/>
        <PerformTask/>
      </Sequence>
      <RecoverySubtree/>
    </ReactiveSequence>
  </BehaviorTree>
  
  <BehaviorTree ID="RecoverySubtree">
    <Fallback>
      <ResetComponent/>
      <EmergencyStop/>
    </Fallback>
  </BehaviorTree>
</root>
```

# Reference
- https://youtu.be/KeShMInMjro?si=9u9t1mcYriB9Qd1w
- https://arxiv.org/pdf/1709.00084
