#ifndef RVIZ_PATH_SEQUENCE_DISPLAY_H
#define RVIZ_PATH_SEQUENCE_DISPLAY_H

#include <path_msgs/PathSequence.h>

#include "rviz/message_filter_display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BillboardLine;
class VectorProperty;


/**
 * \class PathDisplay
 * \brief Displays a path_msgs::PathSequence message
 */
class PathSequenceDisplay: public MessageFilterDisplay<path_msgs::PathSequence>
{
Q_OBJECT
public:
  PathSequenceDisplay();
  virtual ~PathSequenceDisplay();

  /** @brief Overridden from Display. */
  virtual void reset();

protected:
  /** @brief Overridden from Display. */
  virtual void onInitialize();

  /** @brief Overridden from MessageFilterDisplay. */
  void processMessage( const path_msgs::PathSequence::ConstPtr& msg );

private Q_SLOTS:
  void updateBufferLength();
  void updateStyle();
  void updateLineWidth();
  void updateOffset();

private:
  void destroyObjects();

  std::vector<std::vector<Ogre::ManualObject*>> manual_objects_;
  std::vector<std::vector<rviz::BillboardLine*>> billboard_lines_;

  EnumProperty* style_property_;
  ColorProperty* color_forward_property_;
  ColorProperty* color_backward_property_;
  FloatProperty* alpha_property_;
  FloatProperty* line_width_property_;
  IntProperty* buffer_length_property_;
  VectorProperty* offset_property_;

  enum LineStyle {
    LINES,
    BILLBOARDS
  };

};

} // namespace rviz

#endif /* RVIZ_PATH_DISPLAY_H */

