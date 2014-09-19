#!/home/buck/ws/apex/build/devel/lib/csapex/csapex_node
view_pos:
  []
uuid_map:
  csapex::DynamicTransform: 3
  csapex::ExportRos: 3
  csapex::ImageToMap: 2
  csapex::ImportRos: 2
  csapex::MapMerger: 1
  csapex::OutputDisplay: 2
  csapex::ScanFilter: 2
  csapex::ScanRenderer: 1
  csapex::ScanToMap: 2
  csapex::StaticTransform: 2
  vision_plugins::ColorConvert: 1
connections:
  - uuid: csapex::ImportRos_1:|:out_0
    targets:
      - csapex::ScanToMap_1:|:in_0
  - uuid: csapex::ScanToMap_1:|:out_0
    targets:
      - csapex::MapMerger_1:|:in_0
  - uuid: csapex::ImportRos_2:|:out_0
    targets:
      - csapex::ScanToMap_2:|:in_0
  - uuid: csapex::MapMerger_1:|:out_0
    targets:
      - csapex::ExportRos_2:|:in_0
  - uuid: csapex::ScanToMap_2:|:out_0
    targets:
      - csapex::MapMerger_1:|:in_1
  - uuid: csapex::DynamicTransform_2:|:out_0
    targets:
      - csapex::ScanToMap_2:|:in_1
  - uuid: csapex::DynamicTransform_3:|:out_0
    targets:
      - csapex::ScanToMap_1:|:in_1
fulcrums:
  []
adapters:
  []
---
type: csapex::ImportRos
uuid: csapex::ImportRos_1
label: csapex::ImportRos_1
pos:
  - -180
  - 10
minimized: false
enabled: true
flipped: false
state:
  params:
    refresh:
      name: refresh
      type: value
    topic:
      name: topic
      type: set
      txt: /scan/front/filtered
      string: /scan/front/filtered
---
type: csapex::ExportRos
uuid: csapex::ExportRos_2
label: csapex::ExportRos_2
pos:
  - 1040
  - 120
minimized: false
enabled: true
flipped: false
state:
  params:
    topic:
      name: topic
      type: value
      string: /obstacle_map
---
type: csapex::ScanToMap
uuid: csapex::ScanToMap_1
label: csapex::ScanToMap_1
pos:
  - 370
  - -10
minimized: false
enabled: true
flipped: false
state:
  params:
    default:
      name: default
      type: range
      int: 0
    drawRays:
      name: drawRays
      type: value
      bool: true
    frame_id:
      name: frame_id
      type: value
      string: /base_link
    height:
      name: height
      type: range
      int: 200
    resolution:
      name: resolution
      type: range
      double: 0.05
    width:
      name: width
      type: range
      int: 200
    yaw:
      name: yaw
      type: range
      double: 0
---
type: csapex::ImportRos
uuid: csapex::ImportRos_2
label: csapex::ImportRos_2
pos:
  - -180
  - 280
minimized: false
enabled: true
flipped: false
state:
  params:
    refresh:
      name: refresh
      type: value
    topic:
      name: topic
      type: set
      txt: /scan/back/filtered
      string: /scan/back/filtered
---
type: csapex::MapMerger
uuid: csapex::MapMerger_1
label: csapex::MapMerger_1
pos:
  - 720
  - 180
minimized: false
enabled: true
flipped: false
state:
  params:
    offset/x:
      name: offset/x
      type: range
      double: 0
    offset/y:
      name: offset/y
      type: range
      double: 0
---
type: csapex::ScanToMap
uuid: csapex::ScanToMap_2
label: csapex::ScanToMap_1
pos:
  - 370
  - 260
minimized: false
enabled: true
flipped: false
state:
  params:
    default:
      name: default
      type: range
      int: 0
    drawRays:
      name: drawRays
      type: value
      bool: true
    frame_id:
      name: frame_id
      type: value
      string: /base_link
    height:
      name: height
      type: range
      int: 200
    resolution:
      name: resolution
      type: range
      double: 0.05
    width:
      name: width
      type: range
      int: 200
    yaw:
      name: yaw
      type: range
      double: -3.141592653589793
---
type: csapex::DynamicTransform
uuid: csapex::DynamicTransform_2
label: csapex::DynamicTransform_2
pos:
  - -20
  - 380
minimized: false
enabled: true
flipped: false
state:
  params:
    from:
      name: from
      type: set
      txt: /laser_back
      string: /laser_back
    refresh:
      name: refresh
      type: value
    reset tf:
      name: reset tf
      type: value
    to:
      name: to
      type: set
      txt: /base_link
      string: /base_link
---
type: csapex::DynamicTransform
uuid: csapex::DynamicTransform_3
label: csapex::DynamicTransform_2
pos:
  - -20
  - 110
minimized: false
enabled: true
flipped: false
state:
  params:
    from:
      name: from
      type: set
      txt: /laser_front
      string: /laser_front
    refresh:
      name: refresh
      type: value
    reset tf:
      name: reset tf
      type: value
    to:
      name: to
      type: set
      txt: /base_link
      string: /base_link