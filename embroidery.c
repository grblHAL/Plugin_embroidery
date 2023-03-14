/*

  embroidery.c - plugin for reading and executing embroidery files from SD card.

  Copyright (c) 2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

#include "embroidery.h"

#define STITCH_QUEUE_SIZE 8 // must be a power of 2

extern bool brother_open_file (vfs_file_t *file, embroidery_t *api);
extern bool tajima_open_file (vfs_file_t *file, embroidery_t *api);

typedef struct {
    float feedrate;
    float z_travel;
    uint8_t port;
    bool sync_mode;
    uint16_t stop_delay;
} embroidery_settings_t;

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    stitch_t stitch[STITCH_QUEUE_SIZE];
} stitch_queue_t;

typedef struct {
    bool enqueued;
    bool completed;
    bool paused;
    bool stitching;
    bool first;
    volatile bool await_trigger;
    uint32_t trigger_interval;
    uint32_t last_trigger;
    uint32_t stitch_interval;
    uint32_t errs, stitches, queued, exced;
    uint32_t spindle_stop;
    spindle_state_t spindle;
    sys_state_t machine_state;
    vfs_file_t *file;
    plan_line_data_t plan_data;
    coord_data_t position;
    uint8_t color;
    stitch_queue_t queue;
} embroidery_job_t;

static uint8_t port, n_ports;
static char max_port[4];
static uint32_t nvs_address;

static io_stream_t active_stream;
static embroidery_t api;
static embroidery_settings_t embroidery;
static on_report_options_ptr on_report_options;
static on_state_change_ptr on_state_change;
static on_execute_realtime_ptr on_execute_realtime;
static on_file_open_ptr on_file_open;
static driver_reset_ptr driver_reset;
static embroidery_job_t job = {0};

static void end_job (void)
{
    job.completed = job.enqueued = true;
    job.queue.head = job.queue.tail = 0;

    if(active_stream.type != StreamType_Null) {
        memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));
        active_stream.type = StreamType_Null;
    }

    if(job.file) {
        vfs_close(job.file);
        job.file = NULL;
    }

    if(job.spindle.on) {
        job.spindle.on = Off;
        job.plan_data.spindle.hal->set_state(job.spindle, 0.0f);
    }
}

static void thread_change (sys_state_t state)
{
    const char *thread_color = api.get_thread_color(job.color);

    if(job.spindle.on) {
        job.spindle.on = Off;
        job.plan_data.spindle.hal->set_state(job.spindle, 0.0f);
    }

    report_message(thread_color, Message_Info);
    protocol_buffer_synchronize();              // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime();                // Execute suspend.
}

static void thread_trim (sys_state_t state)
{
    if(job.spindle.on) {
        job.spindle.on = Off;
        job.plan_data.spindle.hal->set_state(job.spindle, 0.0f);
    }

    report_message("trim", Message_Info);
    protocol_buffer_synchronize();              // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime();                // Execute suspend.
}

static void onStateChanged (sys_state_t state)
{
    switch(state) {

        case STATE_IDLE:
            if(job.stitching) {
                if(job.first)
                    job.first = false;
                else {
                    uint32_t ms = hal.get_elapsed_ticks() - job.last_trigger;
                    job.stitch_interval = max(job.stitch_interval, ms);
                }
            }
            break;

        case STATE_CYCLE:
//            last_ms = hal.get_elapsed_ticks();
            break;

    }

    hal.port.digital_out(0, state == STATE_CYCLE);

    if(job.machine_state == STATE_HOLD)
        job.paused = false;

    job.machine_state = state;

    if(on_state_change)
        on_state_change(state);
}

static void needle_trigger (uint8_t port, bool state)
{
    uint32_t ms = hal.get_elapsed_ticks();

    if(job.await_trigger && ms - job.last_trigger > 15) {

        job.trigger_interval = ms - job.last_trigger;

        if(job.machine_state == STATE_CYCLE) {
            job.errs++;
            return;
        }

        job.await_trigger = false;
    }

    job.stitches++;
    job.last_trigger = ms;
}

static void onExecuteRealtime (sys_state_t state)
{
    static bool busy = false;

    on_execute_realtime(state);

    if(job.spindle_stop && hal.get_elapsed_ticks() - job.last_trigger >= job.spindle_stop) {
        job.spindle.on = Off;
        job.plan_data.spindle.hal->set_state(job.spindle, 0.0f);
        job.spindle_stop = 0;
    }

    if(busy || job.paused || job.await_trigger)
        return;

    if(job.queue.tail == job.queue.head) {

        if(job.enqueued && !job.completed) {

            end_job();
            hal.stream.cancel_read_buffer();

            if(grbl.on_program_completed)
                grbl.on_program_completed(ProgramFlow_CompletedM30, false);

            grbl.report.feedback_message(Message_ProgramEnd);
        }

        return;
    }

    if(plan_get_block_buffer_available() < 3)
        return;

    stitch_t *stitch = &job.queue.stitch[job.queue.tail];

    // Wait for non-stitching moves to complete before starting stitching
    if(!job.stitching && stitch->type == Stitch_Normal && job.machine_state != STATE_IDLE)
        return;

    busy = true;

    job.queue.tail = ++job.queue.tail & (STITCH_QUEUE_SIZE - 1);

    // If stitching look-ahead to next command to see if we should stop the motor early to avoid overshoot.
    if(job.stitching) {
        if(job.queue.tail != job.queue.head && job.queue.stitch[job.queue.tail].type != Stitch_Normal && embroidery.stop_delay)
            job.spindle_stop = embroidery.stop_delay;
    }

    if(!(job.stitching = stitch->type == Stitch_Normal) && embroidery.stop_delay == 0) {
        job.spindle.on = Off;
        job.plan_data.spindle.hal->set_state(job.spindle, 0.0f);
        job.spindle_stop = 0;
    }

    switch(stitch->type) {

        case Stitch_Normal:
            job.exced++;
            job.plan_data.condition.rapid_motion = Off;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;

            if((job.first = !job.spindle.on)) {
                job.spindle.on = On;
                job.plan_data.spindle.hal->set_state(job.spindle, 1.0f);
            }

            mc_line(job.position.values, &job.plan_data);

            if(!(job.await_trigger = embroidery.sync_mode)) {
//                plan_data.condition.rapid_motion = On;
                job.position.z = -embroidery.z_travel;
                mc_line(job.position.values, &job.plan_data);
                job.position.z = embroidery.z_travel;
                mc_line(job.position.values, &job.plan_data);
            }

            break;

        case Stitch_Jump:
            job.plan_data.condition.rapid_motion = On;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;

            mc_line(job.position.values, &job.plan_data);
            break;

        case Stitch_Trim:
            job.paused = true;
            job.plan_data.condition.rapid_motion = On;
            job.spindle_stop = embroidery.stop_delay;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;
            mc_line(job.position.values, &job.plan_data);

            protocol_enqueue_rt_command(thread_trim);
            break;

        case Stitch_Stop:
            job.paused = true;
            job.plan_data.condition.rapid_motion = On;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;
//            mc_line(job.position.values, &job.plan_data);

            job.color = stitch->color;
            protocol_enqueue_rt_command(thread_change);
            job.spindle_stop = embroidery.stop_delay;
            break;

        case Stitch_SequinEject:
            //??
            break;
    }

    busy = false;
}

static int16_t sdcard_read (void)
{
    if(!job.enqueued) {

        uint_fast8_t bptr = (job.queue.head + 1) & (STITCH_QUEUE_SIZE - 1);

        if(bptr != job.queue.tail) {
            job.enqueued = !api.get_stitch(&job.queue.stitch[job.queue.head], job.file);
            job.queue.head = bptr;
            job.queued++;
        }
    }

    return SERIAL_NO_DATA;
}

static const char *trim (const char *s)
{
    char *s1 = strrchr(s, '\0');

    while(*(--s1) == '0')
        *s1 = '\0';

    if(*s1 == '.')
        *s1 = '\0';

    return s;
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    static spindle_data_t spindle_data = {0};

    if(request == SpindleData_RPM) {
        if(job.spindle.on)
            spindle_data.rpm = 60.0f / (float)job.trigger_interval;
    }

    return &spindle_data;
}

static status_code_t onFileOpen (const char *fname, vfs_file_t *file, bool stream)
{
    bool ok = false;

//    stitch.trims = stitch.colors = 0;
//    stitch.await_trigger = false;

    if(brother_open_file(file, &api) || tajima_open_file(file, &api)) {

        if(stream) {

            memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Save current stream pointers
            hal.stream.type = StreamType_File;                          // then redirect to read from SD card instead
            hal.stream.read = sdcard_read;                              // ...

            job.file = file;
            job.completed = job.enqueued = job.await_trigger = job.paused = false;
            job.queue.head = job.queue.tail = job.stitch_interval = 0;
            job.plan_data.feed_rate = embroidery.feedrate;
            job.plan_data.condition.rapid_motion = On;
            job.plan_data.spindle.hal = gc_state.spindle.hal;
            job.plan_data.spindle.hal->get_data = spindleGetData;
            system_convert_array_steps_to_mpos(job.position.values, sys.position);

            job.errs = job.stitches = job.queued = job.exced = 0;

        } else {

            bool no_move = false;
            coord_data_t target = {0};
            stich_type_t mode = Stitch_Stop;
            stitch_t stitch;

            stitch.target.x = stitch.target.y = 0.0f;

            hal.stream.write("G17G21G91" ASCII_EOL);
            hal.stream.write("F");
            hal.stream.write(uitoa((uint32_t)embroidery.feedrate));
            hal.stream.write(ASCII_EOL);

            while(api.get_stitch(&stitch, file)) {

                if(stitch.type == Stitch_Stop) {
                    hal.stream.write("T");
                    hal.stream.write(uitoa(stitch.color));
                    hal.stream.write(" (MSG,");
                    hal.stream.write(api.get_thread_color(stitch.color));
                    hal.stream.write(")" ASCII_EOL);
                } else if (stitch.type != Stitch_SequinEject) {

                    no_move = target.x == 0.0f && target.y == 0.0f;

                    if(no_move || mode != stitch.type) {
                        hal.stream.write(stitch.type == Stitch_Jump ? "G0" : "G1");
                        mode = stitch.type;
                    }
                    if(stitch.target.x != 0.0f) {
                        hal.stream.write("X");
                        hal.stream.write(trim(ftoa(stitch.target.x, N_DECIMAL_COORDVALUE_MM)));
                        target.x = stitch.target.x;
                    }
                    if(stitch.target.y != 0.0f) {
                        hal.stream.write("Y");
                        hal.stream.write(trim(ftoa(stitch.target.y, N_DECIMAL_COORDVALUE_MM)));
                        target.y = stitch.target.y;
                    }
                    hal.stream.write(ASCII_EOL);

                    if(stitch.type == Stitch_Trim)
                        hal.stream.write("M0 (MSG,Trim thread)" ASCII_EOL);
                }
            }

            hal.stream.write("M30" ASCII_EOL);
            end_job();
        }

        ok = true;
    }

    return ok ? Status_OK : (on_file_open ? on_file_open(fname, file, stream) : Status_Unhandled);
}

static void sdcard_reset (void)
{
    end_job();
    driver_reset();
}

static bool is_setting_available (const setting_detail_t *setting)
{
    return setting->id == Setting_UserDefined_2 && ioport_can_claim_explicit();
}

static const setting_detail_t embroidery_settings[] = {
    { Setting_UserDefined_0, Group_General, "Embroidery feedrate", "mm/min", Format_Decimal, "####0.0", NULL, NULL, Setting_NonCore, &embroidery.feedrate, NULL, NULL },
    { Setting_UserDefined_1, Group_General, "Embroidery Z travel", "mm", Format_Decimal, "##0.0", NULL, NULL, Setting_NonCore, &embroidery.z_travel, NULL, NULL },
    { Setting_UserDefined_2, Group_AuxPorts, "Embroidery trigger port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &embroidery.port, NULL, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_3, Group_General, "Sync mode", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &embroidery.sync_mode, NULL, NULL },
    { Setting_UserDefined_4, Group_General, "Stop delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_NonCore, &embroidery.stop_delay, NULL, NULL }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t embroidery_settings_descr[] = {
    { Setting_UserDefined_0, "Step jogging speed in millimeters per minute." },
    { Setting_UserDefined_1, "Slow jogging speed in millimeters per minute." },
    { Setting_UserDefined_2, "Fast jogging speed in millimeters per minute." }
};

#endif

static void embroidery_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&embroidery, sizeof(embroidery_settings_t), true);
}

static void embroidery_settings_restore (void)
{
    embroidery.feedrate = 4000.0f;
    embroidery.z_travel = 10.0f;
    embroidery.stop_delay = 0;

    if(ioport_can_claim_explicit()) {

        xbar_t *portinfo;
        uint8_t port = n_ports;

        embroidery.port = 0;

        // Find highest numbered port that supports change interrupt.
        if(port > 0) do {
            port--;
            if((portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, port))) {
                if(!portinfo->cap.claimed && (portinfo->cap.irq_mode & IRQ_Mode_Rising)) {
                    embroidery.port = port;
                    break;
                }
            }
        } while(port);
    }

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&embroidery, sizeof(embroidery_settings_t), true);
}

static void warning_pin (uint_fast16_t state)
{
    report_message("Embroidery plugin failed to initialize, no pin for needle trigger signal!", Message_Warning);
}

static void embroidery_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&embroidery, nvs_address, sizeof(embroidery_settings_t), true) != NVS_TransferResult_OK)
        embroidery_settings_restore();

    port = embroidery.port;

    xbar_t *portinfo = hal.port.get_pin_info(Port_Digital, Port_Input, port);

    if(portinfo && !portinfo->cap.claimed && (portinfo->cap.irq_mode & IRQ_Mode_Rising) && ioport_claim(Port_Digital, Port_Input, &port, "Embroidery needle trigger"))
        hal.port.register_interrupt_handler(port, IRQ_Mode_Rising, needle_trigger);
    else
        protocol_enqueue_rt_command(warning_pin);
}

static setting_details_t setting_details = {
    .settings = embroidery_settings,
    .n_settings = sizeof(embroidery_settings) / sizeof(setting_detail_t),
#ifdef NO_SETTINGS_DESCRIPTIONS
    .descriptions = embroidery_settings_descr,
    .n_descriptions = sizeof(embroidery_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = embroidery_settings_load,
    .restore = embroidery_settings_restore,
    .save = embroidery_settings_save
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:EMBROIDERY v0.01]" ASCII_EOL);
}

void embroidery_init (void)
{
    bool ok = false;

   if(!ioport_can_claim_explicit()) {

        // Driver does not support explicit pin claiming, claim the highest numbered port instead.

        if((ok = (n_ports = hal.port.num_digital_in) > 0 && (nvs_address = nvs_alloc(sizeof(embroidery_settings_t)))))
            embroidery.port = --hal.port.num_digital_in;

    } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > 0 && (nvs_address = nvs_alloc(sizeof(embroidery_settings_t)))))
        strcpy(max_port, uitoa(n_ports - 1));

    if(ok) {

        active_stream.type = StreamType_Null;

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = sdcard_reset;

        on_file_open = grbl.on_file_open;
        grbl.on_file_open = onFileOpen;

        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = onExecuteRealtime;

    } else
        protocol_enqueue_rt_command(warning_pin);
}
