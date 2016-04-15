/*
 * filter_tracker.cpp -- animate color to the audio
 * Copyright (C) 2015 Meltytech, LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <framework/mlt.h>
#include <stdlib.h> // calloc(), free()
#include <opencv2/tracking.hpp>


struct cv_data
{
        cv::Ptr<cv::Tracker> tracker;
        cv::Rect2d boundingBox;
        bool initialized;
};

typedef struct cv_data *tracker_data;

/** Get the image.
*/
static int filter_get_image( mlt_frame frame, uint8_t **image, mlt_image_format *format, int *width, int *height, int writable )
{
	int error = 0;
	mlt_filter filter = (mlt_filter)mlt_frame_pop_service( frame );

	mlt_properties filter_properties = MLT_FILTER_PROPERTIES( filter );
        mlt_position position = mlt_filter_get_position( filter, frame );
	mlt_position length = mlt_filter_get_length2( filter, frame );

        mlt_service_lock( MLT_FILTER_SERVICE(filter) );
        tracker_data data = (tracker_data) mlt_properties_get_data( filter_properties, "tracker_data", NULL );

        // Clip already analysed, don't re-process
        if( mlt_properties_get_int( filter_properties, "analyse" ) == 0 && mlt_properties_get(filter_properties, "motion_tracking")) {
                mlt_rect rect = mlt_properties_anim_get_rect(filter_properties, "motion_tracking", position, length);
                if( mlt_properties_get_int( filter_properties, "show_rect" ) != 0 ) {
                        // Get the current image
                        *format = mlt_image_rgb24;
                        error = mlt_frame_get_image( frame, image, format, width, height, 1 );
                        cv::Mat cvFrame = cv::Mat(*height, *width, CV_8UC3, *image);
                        mlt_color rect_color = mlt_properties_get_color( filter_properties, "rect_color" );
                        data->boundingBox.x = std::max(rect.x, 1.0);
                        data->boundingBox.y= std::max(rect.y, 1.0);
                        data->boundingBox.width = rect.w;
                        data->boundingBox.height = rect.h;
                        cv::rectangle( cvFrame, data->boundingBox, cv::Scalar( rect_color.r, rect_color.g, rect_color.b ), mlt_properties_get_int( filter_properties, "show_rect" ), 1);
                } else {
                        // do nothing
                        error = mlt_frame_get_image( frame, image, format, width, height, 1 );
                }
        } else {
                // Get the current image
                *format = mlt_image_rgb24;
                error = mlt_frame_get_image( frame, image, format, width, height, 1 );
                cv::Mat cvFrame = cv::Mat(*height, *width, CV_8UC3, *image);
                if (!data->initialized) {
                        if (data->boundingBox.width <1 || data->boundingBox.height <1) {
                                mlt_rect rect = mlt_properties_get_rect(filter_properties, "rect");
                                data->boundingBox.x = std::max(rect.x, 1.0);
                                data->boundingBox.y= std::max(rect.y, 1.0);
                                data->boundingBox.width = rect.w;
                                data->boundingBox.height = rect.h;
                                if (data->boundingBox.width <1) {
                                        data->boundingBox.width = 50;
                                }
                                if (data->boundingBox.height <1) {
                                        data->boundingBox.height = 50;
                                }
                        }
                        if (data->tracker->init( cvFrame, data->boundingBox )) {
                                data->initialized = true;
                        }
                } else {
                        if (data->tracker->update( cvFrame, data->boundingBox ) ) {
                        }
                }
                if( mlt_properties_get_int( filter_properties, "show_rect" ) != 0 ) {
                        mlt_color rect_color = mlt_properties_get_color( filter_properties, "rect_color" );
                        cv::rectangle( cvFrame, data->boundingBox, cv::Scalar( rect_color.r, rect_color.g, rect_color.b ), mlt_properties_get_int( filter_properties, "show_rect" ), 1);
                }
        }

        if( mlt_properties_get_int( filter_properties, "analyse" ) == 1 ) {
                mlt_rect rect;
                rect.x = data->boundingBox.x;
                rect.y = data->boundingBox.y;
                rect.w = data->boundingBox.width;
                rect.h = data->boundingBox.height;
                mlt_properties_anim_set_rect(filter_properties, "motion_tracking", rect, position, length, mlt_keyframe_linear);
        }

        mlt_service_unlock( MLT_FILTER_SERVICE(filter) );
	
	return error;
}

/** Filter processing.
*/
static mlt_frame filter_process( mlt_filter filter, mlt_frame frame )
{
	mlt_frame_push_service( frame, filter );
	mlt_frame_push_get_image( frame, filter_get_image );
	return frame;
}

static void filter_close( mlt_filter filter )
{
	filter->close = NULL;
	filter->parent.close = NULL;
	mlt_service_close( &filter->parent );
}

/** Constructor for the filter.
*/

extern "C" {

mlt_filter filter_tracker_init( mlt_profile profile, mlt_service_type type, const char *id, char *arg )
{
	mlt_filter filter = mlt_filter_new();
        tracker_data data = (tracker_data)calloc( 1, sizeof(struct cv_data) ); 

	if ( filter && data)
	{
		mlt_properties properties = MLT_FILTER_PROPERTIES( filter );

                // Init tracker
                data->tracker = cv::Tracker::create(cv::String("MIL"));
                if( data->tracker == NULL ) {
                    fprintf(stderr, "Tracker initialized FAILED\n");
                } else {
                    fprintf(stderr, "Tracker initialized\n");
                }
                data->initialized = false;
                data->boundingBox.x = 0;
                data->boundingBox.y= 0;
                data->boundingBox.width = 0;
                data->boundingBox.height = 0;
                mlt_properties_set_data( properties, "tracker_data", data, 0, NULL, NULL );

                // Create a unique ID for storing data on the frame
		filter->close = filter_close;
		filter->process = filter_process;
	}
	else
	{
		mlt_log_error( MLT_FILTER_SERVICE(filter), "Filter tracker failed\n" );

		if( filter )
		{
			mlt_filter_close( filter );
		}

		/*if( pdata )
		{
			free( pdata );
		}*/

		filter = NULL;
	}
	return filter;
}

}

