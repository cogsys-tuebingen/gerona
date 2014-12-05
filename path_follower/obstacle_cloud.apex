#!/localhome/widmaier/ws/misc/devel/lib/csapex/csapex_node
nodes:
  - state:
      params:
        buffer/length:
          max: 10
          name: buffer/length
          double: 1
          min: 0
          type: range
          step: 0.1
        queue:
          min: 0
          max: 30
          name: queue
          type: range
          step: 1
          int: 1
        latch:
          bool: false
          type: value
          name: latch
        topic:
          name: topic
          string: /scan/front/filtered
          txt: /scan/front/filtered
          type: set
        rate:
          type: range
          step: 0.1
          name: rate
          double: 60
          max: 100
          min: 0.1
        refresh:
          name: refresh
          type: trigger
        buffer/max_wait:
          min: 0
          step: 0.1
          name: buffer/max_wait
          type: range
          max: 10
          double: 1
      persistent_params:
        []
    enabled: true
    label: csapex::ImportRos_1
    flipped: false
    type: csapex::ImportRos
    minimized: false
    pos:
      - -132
      - 19
    thread: 0
    uuid: csapex::ImportRos_1
  - flipped: false
    enabled: true
    pos:
      - 412
      - 108
    uuid: csapex::ScanToPointCloud_1
    label: csapex::ScanToPointCloud_1
    type: csapex::ScanToPointCloud
    state:
      persistent_params:
        []
      params:
        frame:
          string: /laser
          type: value
          name: frame
    thread: 0
    minimized: false
  - type: csapex::ExportRos
    pos:
      - 736
      - 168
    thread: 0
    enabled: true
    uuid: csapex::ExportRos_4
    state:
      params:
        topic:
          string: /obstacle_cloud
          type: value
          name: topic
      persistent_params:
        []
    label: csapex::ExportRos_4
    minimized: false
    flipped: false
connections:
  - targets:
      - csapex::ScanToPointCloud_1:|:in_0
    uuid: csapex::ImportRos_1:|:out_0
  - targets:
      - csapex::ExportRos_4:|:in_0
    uuid: csapex::ScanToPointCloud_1:|:out_0
uuid_map:
  csapex::ExportRos: 4
  vision_plugins::ColorConvert: 1
  csapex::StaticTransform: 2
  csapex::OutputDisplay: 2
  csapex::ScanRenderer: 1
  csapex::ScanFilter: 2
  csapex::DynamicTransform: 3
  csapex::ImageToMap: 2
  csapex::MapMerger: 1
  csapex::ImportRos: 2
  csapex::ScanToPointCloud: 1
  csapex::ScanToMap: 2
threads:
  groups: ~
  next_id: 1
  assignments: ~