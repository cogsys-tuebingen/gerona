#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMatrix4.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"

#include "rviz/ogre_helpers/billboard_line.h"
#include "path_display.h"

namespace rviz
{

PathSequenceDisplay::PathSequenceDisplay()
{
    style_property_ = new EnumProperty( "Line Style", "Lines",
                                        "The rendering operation to use to draw the grid lines.",
                                        this, SLOT( updateStyle() ));
    
    style_property_->addOption( "Lines", LINES );
    style_property_->addOption( "Billboards", BILLBOARDS );
    
    line_width_property_ = new FloatProperty( "Line Width", 0.03,
                                              "The width, in meters, of each path line."
                                              "Only works with the 'Billboards' style.",
                                              this, SLOT( updateLineWidth() ), this );
    line_width_property_->setMin( 0.001 );
    line_width_property_->hide();

    color_forward_property_ = new ColorProperty( "Color Forward", QColor( 25, 255, 0 ),
                                                 "Color to draw the path segements that should be traversed forward.", this );
    color_backward_property_ = new ColorProperty( "Color Backward", QColor( 255, 25, 0 ),
                                                  "Color to draw the path segements that should be traversed backward.", this );
    
    alpha_property_ = new FloatProperty( "Alpha", 1.0,
                                         "Amount of transparency to apply to the path.", this );
    
    buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                               "Number of paths to display.",
                                               this, SLOT( updateBufferLength() ));
    buffer_length_property_->setMin( 1 );
    
    offset_property_ = new VectorProperty( "Offset", Ogre::Vector3::ZERO,
                                           "Allows you to offset the path from the origin of the reference frame.  In meters.",
                                           this, SLOT( updateOffset() ));
}

PathSequenceDisplay::~PathSequenceDisplay()
{
    destroyObjects();
}

void PathSequenceDisplay::onInitialize()
{
    MFDClass::onInitialize();
    updateBufferLength();
}

void PathSequenceDisplay::reset()
{
    MFDClass::reset();
    updateBufferLength();
}


void PathSequenceDisplay::updateStyle()
{
    LineStyle style = (LineStyle) style_property_->getOptionInt();
    
    switch( style )
    {
    case LINES:
    default:
        line_width_property_->hide();
        break;
        
    case BILLBOARDS:
        line_width_property_->show();
        break;
    }
    
    updateBufferLength();
}

void PathSequenceDisplay::updateLineWidth()
{
    LineStyle style = (LineStyle) style_property_->getOptionInt();
    float line_width = line_width_property_->getFloat();
    
    if(style == BILLBOARDS) {
        for( size_t k = 0; k < billboard_lines_.size(); k++ )
        {
            std::vector<rviz::BillboardLine*>& billboard_lines = billboard_lines_[ k ];
            for( size_t i = 0; i < billboard_lines.size(); i++ )
            {
                rviz::BillboardLine* billboard_line = billboard_lines[ i ];
                if( billboard_line ) billboard_line->setLineWidth( line_width );
            }
        }
        context_->queueRender();
    }
}

void PathSequenceDisplay::updateOffset()
{
    scene_node_->setPosition( offset_property_->getVector() );
    context_->queueRender();
}

void PathSequenceDisplay::destroyObjects()
{
    // Destroy all simple lines, if any
    for( size_t k = 0; k < manual_objects_.size(); k++ )
    {
        std::vector<Ogre::ManualObject*>& manual_objects = manual_objects_[k];
        for( size_t i = 0; i < manual_objects.size(); i++ )
        {
            Ogre::ManualObject*& manual_object = manual_objects[ i ];
            if( manual_object )
            {
                manual_object->clear();
                scene_manager_->destroyManualObject( manual_object );
                manual_object = NULL; // ensure it doesn't get destroyed again
            }
        }
    }

    // Destroy all billboards, if any
    for( size_t k = 0; k < billboard_lines_.size(); k++ )
    {
        std::vector<rviz::BillboardLine*>& billboard_lines = billboard_lines_[ k ];
        for( size_t i = 0; i < billboard_lines.size(); i++ )
        {
            rviz::BillboardLine*& billboard_line = billboard_lines[ i ];
            if( billboard_line )
            {
                delete billboard_line; // also destroys the corresponding scene node
                billboard_line = NULL; // ensure it doesn't get destroyed again
            }
        }
    }
}

void PathSequenceDisplay::updateBufferLength()
{
    // Delete old path objects
    destroyObjects();

    // Read options
    int buffer_length = buffer_length_property_->getInt();
    LineStyle style = (LineStyle) style_property_->getOptionInt();

    // Create new path objects
    switch(style)
    {
    case LINES: // simple lines with fixed width of 1px
        manual_objects_.resize( buffer_length );
        break;

    case BILLBOARDS: // billboards with configurable width
        billboard_lines_.resize( buffer_length );
        break;
    }


}

bool validateFloats( const path_msgs::PathSequence& msg )
{
    bool valid = true;
    for(const path_msgs::DirectionalPath& dp : msg.paths) {
        valid = valid && validateFloats( dp.poses );
    }
    return valid;
}

void PathSequenceDisplay::processMessage( const path_msgs::PathSequence::ConstPtr& msg )
{
    // Calculate index of oldest element in cyclic buffer
    size_t bufferIndex = messages_received_ % buffer_length_property_->getInt();

    LineStyle style = (LineStyle) style_property_->getOptionInt();
    std::vector<Ogre::ManualObject*>* manual_objects = NULL;
    std::vector<rviz::BillboardLine*>* billboard_lines = NULL;

    // Delete oldest element
    switch(style)
    {
    case LINES:
        manual_objects = &manual_objects_[ bufferIndex ];
        for(Ogre::ManualObject* mo : *manual_objects) {
            mo->clear();
        }
        break;

    case BILLBOARDS:
        billboard_lines = &billboard_lines_[ bufferIndex ];
        for(rviz::BillboardLine* bi : *billboard_lines) {
            bi->clear();
        }
        break;
    }
    // Check if path contains invalid coordinate values
    if( !validateFloats( *msg ))
    {
        setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
        return;
    }

    if(msg->paths.empty()) {
        // no warning, this case is allowed to signal that no path exists
        return;
    }

    // Verify the path message's integrity
    for(const path_msgs::DirectionalPath& dp : msg->paths) {
        if(dp.poses.empty()) {
            // if there is at least one subpath, then all of them must be valid and have at least one way point
            setStatus( StatusProperty::Error, "Topic", "Message contained invalid sub paths of length 0" );
            return;
        }
    }

    // Create new path objects
    std::size_t n = msg->paths.size();
    switch(style)
    {
    case LINES: // simple lines with fixed width of 1px
        for( size_t i = manual_objects->size(); i < n; i++ )
        {
            Ogre::ManualObject* manual_object = scene_manager_->createManualObject();
            manual_object->setDynamic( true );
            scene_node_->attachObject( manual_object );

            manual_objects->push_back(manual_object);
        }
        break;

    case BILLBOARDS: // billboards with configurable width
        for( size_t i = billboard_lines->size(); i < n; i++ )
        {
            rviz::BillboardLine* billboard_line = new rviz::BillboardLine(scene_manager_, scene_node_);
            billboard_lines->push_back(billboard_line);
        }
        break;
    }
    // Lookup transform into fixed frame
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
    {
        ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    }

    Ogre::Matrix4 transform( orientation );
    transform.setTrans( position );

    //  scene_node_->setPosition( position );
    //  scene_node_->setOrientation( orientation );

    Ogre::ColourValue color_forward = color_forward_property_->getOgreColor();
    Ogre::ColourValue color_backward = color_backward_property_->getOgreColor();

    color_forward.a = alpha_property_->getFloat();
    color_backward.a = alpha_property_->getFloat();

    float line_width = line_width_property_->getFloat();


    switch(style)
    {
    case LINES:
        for(std::size_t k = 0, n = msg->paths.size(); k < n; ++k) {
            Ogre::ManualObject* manual_object = manual_objects->at(k);
            uint32_t num_points = msg->paths[k].poses.size();
            if(k > 0) {
                // for later segments we need to re-render the last point of the preceeding path
                ++num_points;
            }

            manual_object->estimateVertexCount( num_points );
            manual_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );

            bool forward = msg->paths[k].forward;
            const auto& color = forward ? color_forward : color_backward;

            if(k > 0) {
                // for later segments we need to re-render the last point of the preceeding path
                const geometry_msgs::Point& pos = msg->paths[k-1].poses.back().pose.position;
                Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                manual_object->position( xpos.x, xpos.y, xpos.z );
                manual_object->colour(color);
            }

            for(const geometry_msgs::PoseStamped& pose : msg->paths[k].poses)
            {
                const geometry_msgs::Point& pos = pose.pose.position;
                Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                manual_object->position( xpos.x, xpos.y, xpos.z );
                manual_object->colour(color);
            }
            manual_object->end();
        }
        break;

    case BILLBOARDS:
        for(std::size_t k = 0, n = msg->paths.size(); k < n; ++k) {
            rviz::BillboardLine* billboard_line = billboard_lines->at(k);
            uint32_t num_points = msg->paths[k].poses.size();
            if(k > 0) {
                // for later segments we need to re-render the last point of the preceeding path
                ++num_points;
            }

            billboard_line->setNumLines( 1 );
            billboard_line->setMaxPointsPerLine( num_points );
            billboard_line->setLineWidth( line_width );

            bool forward = msg->paths[k].forward;
            const auto& color = forward ? color_forward : color_backward;

            if(k > 0) {
                // for later segments we need to re-render the last point of the preceeding path
                const geometry_msgs::Point& pos = msg->paths[k-1].poses.back().pose.position;
                Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                billboard_line->addPoint( xpos, color );
            }
            for(const geometry_msgs::PoseStamped& pose : msg->paths[k].poses)
            {
                const geometry_msgs::Point& pos = pose.pose.position;
                Ogre::Vector3 xpos = transform * Ogre::Vector3( pos.x, pos.y, pos.z );
                billboard_line->addPoint( xpos, color );
            }
        }

        break;
    }

}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::PathSequenceDisplay, rviz::Display )
