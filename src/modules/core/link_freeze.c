/*
 * link_freeze.c
 * Copyright (C) 2020-2025 Meltytech, LLC
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <framework/mlt_factory.h>
#include <framework/mlt_frame.h>
#include <framework/mlt_link.h>
#include <framework/mlt_log.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Private Types
typedef struct
{
    mlt_position freeze_position;
    mlt_frame freeze_frame;
    int freeze_after;
    int freeze_before;
} private_data;

static void property_changed(mlt_service owner, mlt_link self, mlt_event_data event_data)
{
    const char *name = mlt_event_data_to_string(event_data);

    if (!name)
        return;

    if (strcmp("freeze_position", name) == 0) {
        // Freeze position changed
        private_data *pdata = (private_data *) self->child;
        pdata->freeze_position = mlt_properties_get_position(MLT_LINK_PROPERTIES(self), "freeze_position");
        pdata->freeze_frame = NULL;
    } else if (strcmp("freeze_before", name) == 0) {
        // Param change
        private_data *pdata = (private_data *) self->child;
        pdata->freeze_before = mlt_properties_get_int(MLT_LINK_PROPERTIES(self), "freeze_before");
    } else if (strcmp("freeze_after", name) == 0) {
        // Param change
        private_data *pdata = (private_data *) self->child;
        pdata->freeze_after = mlt_properties_get_int(MLT_LINK_PROPERTIES(self), "freeze_after");
    }
}

static int link_get_image(mlt_frame frame,
                                  uint8_t **image,
                                  mlt_image_format *format,
                                  int *width,
                                  int *height,
                                  int writable)
{
    mlt_link self = (mlt_link) mlt_frame_pop_get_image(frame);
    mlt_properties unique_properties = mlt_frame_get_unique_properties(frame,
                                                                       MLT_LINK_SERVICE(self));
    if (!unique_properties) {
        return 1;
    }

    mlt_position frame_pos = mlt_frame_get_position(frame);
    private_data *pdata = (private_data *) self->child;

    if (pdata->freeze_frame == NULL) {
        mlt_producer_seek(self->next, pdata->freeze_position);
        int result = mlt_service_get_frame(MLT_PRODUCER_SERVICE(self->next), &pdata->freeze_frame, 0);
    }

    int use_current_frame = 0;
    if (pdata->freeze_after && frame_pos < pdata->freeze_position) {
        use_current_frame = 1;
    } else if (pdata->freeze_before && frame_pos > pdata->freeze_position) {
        use_current_frame = 1;
    }
    if (use_current_frame) {
        int error = mlt_frame_get_image(frame, image, format, width, height, 0);
        return error;

    }
    uint8_t *in_image;
    int error = mlt_frame_get_image(pdata->freeze_frame, &in_image, format, width, height, 0);

    if (!error) {
        int size = mlt_image_format_size(*format, *width, *height, NULL);
        *image = mlt_pool_alloc(size);
        memcpy(*image, in_image, size);
        mlt_frame_set_image(frame, *image, size, mlt_pool_release);
        mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame), "format", *format);
        mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame), "width", *width);
        mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame), "height", *height);
        mlt_properties_set_int(MLT_FRAME_PROPERTIES(frame),
                                "colorspace",
                                mlt_properties_get_int(MLT_FRAME_PROPERTIES(pdata->freeze_frame),
                                                          "colorspace"));

        uint8_t *in_alpha = mlt_frame_get_alpha(pdata->freeze_frame);
        if (in_alpha) {
            size = *width * *height;
            uint8_t *out_alpha = mlt_pool_alloc(size);
            memcpy(out_alpha, in_alpha, size);
            mlt_frame_set_alpha(frame, out_alpha, size, mlt_pool_release);
        };
        return 0;
    }

    return 1;
}

static int link_get_frame(mlt_link self, mlt_frame_ptr frame, int index)
{
    int error = 0;
    mlt_position frame_pos = mlt_producer_position(MLT_LINK_PRODUCER(self));

    mlt_producer_seek(self->next, frame_pos);
    error = mlt_service_get_frame(MLT_PRODUCER_SERVICE(self->next), frame, index);
    mlt_producer original_producer = mlt_frame_get_original_producer(*frame);

        // Pass original producer dimensions with the frame
        mlt_properties unique_properties = mlt_frame_unique_properties(*frame, MLT_LINK_SERVICE(self));
    mlt_properties original_producer_properties = MLT_PRODUCER_PROPERTIES(original_producer);
    if (mlt_properties_exists(original_producer_properties, "width")) {
        mlt_properties_set_int(unique_properties,
                               "width",
                               mlt_properties_get_int(original_producer_properties, "width"));
    } else if (mlt_properties_exists(original_producer_properties, "meta.media.width")) {
        mlt_properties_set_int(unique_properties,
                               "width",
                               mlt_properties_get_int(original_producer_properties,
                                                      "meta.media.width"));
    }
    if (mlt_properties_exists(original_producer_properties, "height")) {
        mlt_properties_set_int(unique_properties,
                               "height",
                               mlt_properties_get_int(original_producer_properties, "height"));
    } else if (mlt_properties_exists(original_producer_properties, "meta.media.height")) {
        mlt_properties_set_int(unique_properties,
                               "height",
                               mlt_properties_get_int(original_producer_properties,
                                                      "meta.media.height"));
    }
    if (mlt_properties_exists(original_producer_properties, "format")) {
        mlt_properties_set_int(unique_properties,
                               "format",
                               mlt_properties_get_int(original_producer_properties, "format"));
    }

    mlt_frame_push_service(*frame, self);
    mlt_frame_push_get_image(*frame, link_get_image);
    mlt_producer_prepare_next(MLT_LINK_PRODUCER(self));

    return error;
}

static void link_close(mlt_link self)
{
    if (self) {
        private_data *pdata = (private_data *) self->child;
        if (pdata) {
            mlt_frame_close(pdata->freeze_frame);
            free(pdata);
        }
        self->close = NULL;
        mlt_link_close(self);
        free(self);
    }
}

mlt_link link_freeze_init(mlt_profile profile, mlt_service_type type, const char *id, char *arg)
{
    mlt_link self = mlt_link_init();
    private_data *pdata = (private_data *) calloc(1, sizeof(private_data));

    if (self && pdata) {
        self->child = pdata;

        // Callback registration
        self->get_frame = link_get_frame;
        self->close = link_close;

        mlt_properties properties = MLT_LINK_PROPERTIES(self);
        mlt_properties_set_int(properties, "freeze_position", 0);
        mlt_properties_set_int(properties, "freeze_before", 0);
        mlt_properties_set_int(properties, "freeze_after", 0);

        mlt_events_listen(properties,
                          self,
                          "property-changed",
                          (mlt_listener) property_changed);
    } else {
        free(pdata);
        mlt_link_close(self);
        self = NULL;
    }
    return self;
}
