#!/localhome/widmaier/catkin/devel/lib/csapex/csapex_node
view_pos:
  []
uuid_map:
  csapex::ExportRos: 3
  csapex::ImageToMap: 2
  csapex::ImportRos: 1
  csapex::OutputDisplay: 2
  csapex::ScanFilter: 2
  csapex::ScanRenderer: 1
  csapex::ScanToMap: 1
  vision_plugins::ColorConvert: 1
connections:
  - uuid: csapex::ImportRos_1:|:out_0
    targets:
      - csapex::ScanToMap_1:|:in_0
  - uuid: csapex::ScanToMap_1:|:out_0
    targets:
      - csapex::ExportRos_2:|:in_0
fulcrums:
  []
adapters:
  []
---
{type: csapex::ImportRos, uuid: csapex::ImportRos_1, label: csapex::ImportRos_1, pos: [0, 0], minimized: false, enabled: true, flipped: false, state: {params: {refresh: {name: refresh, type: value}, topic: {name: topic, type: set, txt: /scan/front/filtered, string: /scan/front/filtered}}}}
---
{type: csapex::ExportRos, uuid: csapex::ExportRos_2, label: csapex::ExportRos_2, pos: [705, 99], minimized: false, enabled: true, flipped: false, state: {params: {topic: {name: topic, type: value, string: /obstacle_map}}}}
---
{type: csapex::ScanToMap, uuid: csapex::ScanToMap_1, label: csapex::ScanToMap_1, pos: [351, -26], minimized: false, enabled: true, flipped: false, state: {params: {default: {name: default, type: range, int: 0}, drawRays: {name: drawRays, type: value, bool: true}, frame_id: {name: frame_id, type: value, string: /laser_front}, height: {name: height, type: range, int: 200}, resolution: {name: resolution, type: range, double: 0.05}, width: {name: width, type: range, int: 200}}}}
