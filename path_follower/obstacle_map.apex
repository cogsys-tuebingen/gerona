#!~/ws/apex/devel/lib/csapex/csapex_node
view_pos:
  []
uuid_map:
  csapex::AdaptiveThreshold: 1
  csapex::BarCodeReader: 1
  csapex::BlobDetector: 1
  csapex::BoxBlur: 1
  csapex::Camera: 4
  csapex::Clock: 4
  csapex::CloudRenderer: 2
  csapex::ColorPointCloud: 1
  csapex::DecisionTreeTrainer: 1
  csapex::Delay: 5
  csapex::DisplayKeypoints: 1
  csapex::DoubleBuffer: 3
  csapex::ExportFile: 1
  csapex::ExportRos: 2
  csapex::ExtractROI: 1
  csapex::FileImporter: 2
  csapex::FilterApplyMask: 1
  csapex::FilterStaticMask: 1
  csapex::Foreach: 1
  csapex::GaussianBlur: 1
  csapex::GrowROI: 1
  csapex::ImportFile: 1
  csapex::ImportRos: 4
  csapex::LabeledScanToScan: 3
  csapex::LegDetector: 1
  csapex::LegFeatures: 1
  csapex::LineFitSegmnetation: 1
  csapex::MapToImage: 1
  csapex::Morpholocial: 4
  csapex::NumberGenerator: 5
  csapex::OutputDisplay: 22
  csapex::P2PLineSegmentation: 1
  csapex::P2PSegmentation: 4
  csapex::PatsyLegDetector: 1
  csapex::Relay: 5
  csapex::RenderROIs: 2
  csapex::SayText: 3
  csapex::ScanLabeler: 2
  csapex::ScanRenderer: 6
  csapex::ScanSegmentFeatureExtractor: 1
  csapex::ScanSegmentation2DRenderer: 3
  csapex::ScanToMap: 1
  csapex::ScanToPointCloud: 1
  csapex::Segmentation: 1
  csapex::SpinelloDataImporter: 2
  csapex::SpinelloPersonDetector: 3
  csapex::TextDisplay: 6
  csapex::TransformCombiner: 1
  vision_plugins::AbsoluteDifference: 1
  vision_plugins::ColorConvert: 3
  vision_plugins::Debayer: 1
  vision_plugins::GenericImageCombiner: 8
  vision_plugins::PerspectiveTransform: 1
  vision_plugins::Undistort: 1
connections:
  - uuid: csapex::ScanToMap_1:|:out_0
    targets:
      - csapex::ExportRos_2:|:in_0
      - csapex::MapToImage_1:|:in_0
  - uuid: csapex::ImportRos_4:|:out_0
    targets:
      - csapex::ScanToMap_1:|:in_0
  - uuid: csapex::MapToImage_1:|:out_0
    targets:
      - csapex::OutputDisplay_22:|:in_0
fulcrums:
  []
adapters:
  - uuid: csapex::OutputDisplay_22
    state:
      width: 1000
      height: 997
---
{type: csapex::ScanToMap, uuid: csapex::ScanToMap_1, label: csapex::ScanToMap_1, pos: [-620, -60], minimized: false, enabled: true, flipped: false, state: {params: {default: {name: default, type: range, int: 0}, drawRays: {name: drawRays, type: value, bool: false}, frame_id: {name: frame_id, type: value, string: /laser}, height: {name: height, type: range, int: 300}, resolution: {name: resolution, type: range, double: 0.05}, width: {name: width, type: range, int: 300}}}}
---
{type: csapex::ImportRos, uuid: csapex::ImportRos_4, label: csapex::ImportRos_4, pos: [-930, -220], minimized: false, enabled: true, flipped: false, state: {params: {refresh: {name: refresh, type: value}, topic: {name: topic, type: set, txt: /scan_filtered, string: /scan_filtered}}}}
---
{type: csapex::ExportRos, uuid: csapex::ExportRos_2, label: csapex::ExportRos_2, pos: [-250, -190], minimized: false, enabled: true, flipped: false, state: {params: {topic: {name: topic, type: value, string: /obstacle_map}}}}
---
{type: csapex::MapToImage, uuid: csapex::MapToImage_1, label: csapex::MapToImage_1, pos: [-40, -60], minimized: false, enabled: true, flipped: false, state: {params: {}}}
---
{type: csapex::OutputDisplay, uuid: csapex::OutputDisplay_22, label: csapex::OutputDisplay_22, pos: [0, 0], minimized: false, enabled: true, flipped: false, state: {params: {}}}
