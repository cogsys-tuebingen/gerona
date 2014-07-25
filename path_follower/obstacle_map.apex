#!/localhome/widmaier/catkin/devel/lib/csapex/csapex_node
window_size:
  - 1600
  - 1151
window_pos:
  - 0
  - 25
view_pos:
  - 0
  - 0
uuid_map:
  csapex::ExportRos: 3
  csapex::ImageToMap: 2
  csapex::ImportRos: 1
  csapex::OutputDisplay: 1
  csapex::ScanFilter: 2
  csapex::ScanRenderer: 1
  vision_plugins::ColorConvert: 1
connections:
  - uuid: csapex::ImportRos_1:|:out_0
    targets:
      - csapex::ScanRenderer_1:|:in_0
  - uuid: csapex::ScanRenderer_1:|:out_0
    targets:
      - csapex::OutputDisplay_1:|:in_0
      - vision_plugins::ColorConvert_1:|:in_0
  - uuid: vision_plugins::ColorConvert_1:|:out_0
    targets:
      - csapex::ImageToMap_2:|:in_0
  - uuid: csapex::ImageToMap_2:|:out_0
    targets:
      - csapex::ExportRos_2:|:in_0
fulcrums:
  - from: csapex::ScanRenderer_1:|:out_0
    to: vision_plugins::ColorConvert_1:|:in_0
    pts: [[728, 378], [144, 391]]
    types: [0, 0]
---
{type: csapex::ImportRos, uuid: csapex::ImportRos_1, label: csapex::ImportRos_1, pos: [0, 0], minimized: false, enabled: true, flipped: false, state: {params: {refresh: {name: refresh, type: value}, topic: {name: topic, type: set, txt: /scan_filtered, string: /scan_filtered}}}}
---
{type: csapex::ScanRenderer, uuid: csapex::ScanRenderer_1, label: csapex::ScanRenderer_1, pos: [488, 7], minimized: false, enabled: true, flipped: false, state: {params: {color/bg: {name: color/bg, type: color, values: [255, 255, 255]}, color/hit: {name: color/hit, type: color, values: [0, 0, 0]}, color/ray: {name: color/ray, type: color, values: [255, 0, 255]}, drawHits: {name: drawHits, type: value, bool: true}, drawRays: {name: drawRays, type: value, bool: false}, height: {name: height, type: range, int: 100}, rotation: {name: rotation, type: range, double: 0}, scale: {name: scale, type: range, double: 1}, width: {name: width, type: range, int: 100}}}}
---
{type: csapex::OutputDisplay, uuid: csapex::OutputDisplay_1, label: csapex::OutputDisplay_1, pos: [929, 0], minimized: false, enabled: true, flipped: false, state: {params: {}}}
---
{type: vision_plugins::ColorConvert, uuid: vision_plugins::ColorConvert_1, label: vision_plugins::ColorConvert_1, pos: [168, 522], minimized: false, enabled: true, flipped: false, state: {params: {input: {name: input, type: set, txt: BGR, int: 2}, output: {name: output, type: set, txt: MONO, int: 5}}}}
---
{type: csapex::ImageToMap, uuid: csapex::ImageToMap_2, label: csapex::ImageToMap_2, pos: [549, 451], minimized: false, enabled: true, flipped: false, state: {params: {frame: {name: frame, type: value, string: laser}, resolution: {name: resolution, type: value, double: 0.1}}}}
---
{type: csapex::ExportRos, uuid: csapex::ExportRos_2, label: csapex::ExportRos_2, pos: [990, 456], minimized: false, enabled: true, flipped: false, state: {params: {topic: {name: topic, type: value, string: obstaclemap}}}}