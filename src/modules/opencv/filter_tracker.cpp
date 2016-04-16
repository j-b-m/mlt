/*
 * filter_tracker.cpp -- Motion tracker
 * Copyright (C) 2016 Jean-Baptiste Mardelle
 * Copyright (C) 2016 Meltytech, LLC
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
#include <stdlib.h>
#include <opencv2/tracking.hpp>


typedef struct
{
        cv::Ptr<cv::Tracker> tracker;
        cv::Rect2d boundingBox;
        char * algo;
        mlt_rect startRect;
        bool initialized;
} private_data;


static void property_changed( mlt_service owner, mlt_filter filter, char *name )
{
	private_data* pdata = (private_data*)filter->child;
        if ( !pdata->initialized )
        {
                return;
        }
	if ( !strcmp( name, "rect" ) )
        {
                mlt_rect rect = mlt_properties_get_rect(MLT_FILTER_PROPERTIES( filter ), "rect");
                if ( rect.x != pdata->startRect.x || rect.y != pdata->startRect.y || rect.w != pdata->startRect.w || rect.h != pdata->startRect.h )
                {
                        pdata->initialized = false;
                }
        }
        else if ( !strcmp( name, "algo" ) )
        {
                char *algo = mlt_properties_get(MLT_FILTER_PROPERTIES( filter ), "algo");
                if ( strcmp( algo, pdata->algo ) )
                {
                        pdata->initialized = false;
                }
        }
        else if ( !strcmp( name, "_reset" ) )
	{
                mlt_properties_set(MLT_FILTER_PROPERTIES( filter ), "results", (char*) NULL );
        }
}

static void apply( mlt_filter filter, private_data* data, int width, int height, int position, int length )
{
	mlt_properties properties = MLT_FILTER_PROPERTIES( filter );
        mlt_rect rect = mlt_properties_anim_get_rect(properties, "results", position, length);
        data->boundingBox.x = std::max(rect.x, 1.0);
        data->boundingBox.y= std::max(rect.y, 1.0);
        data->boundingBox.width = rect.w;
        data->boundingBox.height = rect.h;
}


static void analyse( mlt_filter filter, cv::Mat cvFrame, private_data* data, int width, int height, int position, int length )
{
	mlt_properties filter_properties = MLT_FILTER_PROPERTIES( filter );

        // Create tracker and initialize it
        if (!data->initialized) {

                // Build tracker
                data->algo = mlt_properties_get( filter_properties, "algo" );
                if ( data->algo == NULL || !strcmp(data->algo, "" ) ) {
                        data->tracker = cv::Tracker::create("KCF");
                } else {
                        data->tracker = cv::Tracker::create(data->algo);
                }

                // Discard previous results
                mlt_properties_set(filter_properties, "_results", (char*) NULL );
                if( data->tracker == NULL ) {
                        fprintf(stderr, "Tracker initialized FAILED\n");
                } else {
                        data->startRect = mlt_properties_get_rect(filter_properties, "rect");
                        data->boundingBox.x = std::max(data->startRect.x, 1.0);
                        data->boundingBox.y= std::max(data->startRect.y, 1.0);
                        data->boundingBox.width = data->startRect.w;
                        data->boundingBox.height = data->startRect.h;
                        if (data->boundingBox.width <1) {
                                data->boundingBox.width = 50;
                        }
                        if (data->boundingBox.height <1) {
                                data->boundingBox.height = 50;
                        }
                        if (data->tracker->init( cvFrame, data->boundingBox )) {
                                data->initialized = true;
                        }
                }
        } else {
                data->tracker->update( cvFrame, data->boundingBox );
        }

        // Store results in temp variable
        mlt_rect rect;
        rect.x = data->boundingBox.x;
        rect.y = data->boundingBox.y;
        rect.w = data->boundingBox.width;
        rect.h = data->boundingBox.height;
        rect.o = 0;
        mlt_properties_anim_set_rect(filter_properties, "_results", rect, position, length, mlt_keyframe_linear);
        if ( position == length )
        {
                //Analysis finished, store results
                mlt_properties_set(filter_properties, "results", mlt_properties_get(filter_properties, "_results" ) );
                // Discard temporary data
                mlt_properties_set(filter_properties, "_results", (char*) NULL );
        }
}


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
        int shape_width = mlt_properties_get_int( filter_properties, "shape_width" );
        int blur = mlt_properties_get_int( filter_properties, "blur" );
        cv::Mat cvFrame;
        if ( shape_width == 0 && blur == 0 ) {
                error = mlt_frame_get_image( frame, image, format, width, height, 1 );
        }
        else
        {
                *format = mlt_image_rgb24;
                error = mlt_frame_get_image( frame, image, format, width, height, 1 );
                cvFrame = cv::Mat(*height, *width, CV_8UC3, *image);
        }
        private_data* data = (private_data*)filter->child;

        char* results = mlt_properties_get( filter_properties, "results" );
	if( results && strcmp( results, "" ) )
	{
                // Clip already analysed, don't re-process
                apply( filter, data, *width, *height, position, length );
	}
	else
	{
		analyse( filter, cvFrame, data, *width, *height, position, length );
	}
	
	if ( blur > 0 )
        {
                switch ( mlt_properties_get_int( filter_properties, "blur_type" ) )
                {
                        case 1:
                                // Gaussian Blur
                                cv::GaussianBlur(cvFrame(data->boundingBox), cvFrame(data->boundingBox), cv::Size(0, 0), blur);
                                break;
                        case 0:
                        default:
                                // Median Blur
                                ++blur;
                                if (blur % 2 == 0)
                                {
                                        // median blur param must be odd and, minimum 3
                                        ++blur;
                                }
                                cv::medianBlur( cvFrame( data->boundingBox ), cvFrame( data->boundingBox ), blur );
                                break;
                }
        }

	// Paint overlay shape
	if ( shape_width != 0 ) {
                // Get the OpenCV image
                mlt_color shape_color = mlt_properties_get_color( filter_properties, "shape_color" );
                switch ( mlt_properties_get_int( filter_properties, "shape" ) ) {
                case 2:
                        // Arrow
                        cv::arrowedLine( cvFrame, cv::Point(data->boundingBox.x + data->boundingBox.width/2, data->boundingBox.y - data->boundingBox.height/2), cv::Point(data->boundingBox.x + data->boundingBox.width/2, data->boundingBox.y), cv::Scalar( shape_color.r, shape_color.g, shape_color.b ), std::max( shape_width, 1 ), 4, 0, .2);
                        break;
                case 1:
                        // Ellipse
                        {
                                cv::RotatedRect bounding = cv::RotatedRect(cv::Point2f(data->boundingBox.x + data->boundingBox.width/2, data->boundingBox.y + data->boundingBox.height/2), cv::Size2f(data->boundingBox.width, data->boundingBox.height), 0);
                                cv::ellipse( cvFrame, bounding, cv::Scalar( shape_color.r, shape_color.g, shape_color.b ), shape_width, 1);
                        }
                        break;
                case 0:
                default:
                        // Rectangle
                        cv::rectangle( cvFrame, data->boundingBox, cv::Scalar( shape_color.r, shape_color.g, shape_color.b ), shape_width, 1);
                        break;
        }
        }

        mlt_service_unlock( MLT_FILTER_SERVICE( filter ) );
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
        private_data* data = (private_data*)filter->child;
        free ( data->tracker );
        free ( data );
        filter->child = NULL;
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
        private_data* data = (private_data*)calloc( 1, sizeof(private_data) );

	if ( filter && data)
	{
                mlt_properties properties = MLT_FILTER_PROPERTIES( filter );
                mlt_properties_set_int( properties, "shape_width", 1 );
                mlt_properties_set( properties, "algo", "KCF" );
                data->initialized = false;
                data->boundingBox.x = 0;
                data->boundingBox.y= 0;
                data->boundingBox.width = 0;
                data->boundingBox.height = 0;
                filter->child = data;

                // Create a unique ID for storing data on the frame
		filter->close = filter_close;
		filter->process = filter_process;

                mlt_events_listen( properties, filter, "property-changed", (mlt_listener)property_changed );
	}
	else
	{
		mlt_log_error( MLT_FILTER_SERVICE(filter), "Filter tracker failed\n" );

		if( filter )
		{
			mlt_filter_close( filter );
		}

		if( data )
		{
			free( data );
		}

		filter = NULL;
	}
	return filter;
}

}

