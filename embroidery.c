/*

  embroidery.c - plugin for reading and executing embroidery files from SD card.

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more programmed.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "embroidery.h"

#if EMBROIDERY_ENABLE

#ifndef IOPORT_UNASSIGNED
#define IOPORT_UNASSIGNED 255
#endif

#if !(FS_ENABLE & FS_SDCARD)
#error "Embroidery plugin requires SD card plugin enabled!"
#endif

#include "grbl/task.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

#define STITCH_QUEUE_SIZE 8 // must be a power of 2

extern bool brother_open_file (vfs_file_t *file, embroidery_t *api);
extern bool tajima_open_file (vfs_file_t *file, embroidery_t *api);

typedef enum {
    EmbroideryTrig_Falling = 0,
    EmbroideryTrig_Rising,
    EmbroideryTrig_ZLimit
} embroidery_trig_t;

typedef struct {
    float feedrate;
    float z_travel;
    uint8_t port;
    bool sync_mode;
    uint16_t stop_delay;
    embroidery_trig_t edge;
    uint8_t debug_port;
    uint8_t break_port;
    uint8_t jump_port;
} embroidery_settings_t;

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    stitch_t stitch[STITCH_QUEUE_SIZE];
} stitch_queue_t;

typedef struct {
    uint32_t jumps;
    uint32_t stitches;
    uint32_t trims;
    uint32_t thread_changes;
    uint32_t sequin_ejects;
} embroidery_job_details_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t pause   :1,
                 trigger :1,
                 jump    :1;
    };
} embroidery_await_t;

typedef struct {
    bool enqueued;
    bool completed;
    bool paused;
    bool stitching;
    bool first;
    volatile embroidery_await_t await;
    uint32_t trigger_interval, trigger_interval_min;
    uint32_t last_trigger;
    uint32_t stitch_interval;
    embroidery_job_details_t programmed;
    embroidery_job_details_t executed;
    uint32_t errs, exced, breaks;
    uint32_t spindle_stop;
    spindle_state_t spindle;
    volatile sys_state_t machine_state;
    vfs_file_t *file;
    plan_line_data_t plan_data;
    coord_data_t position;
    embroidery_thread_color_t color;
    stitch_queue_t queue;
} embroidery_job_t;

static uint8_t port, break_port, jump_port, debug_port, n_din, n_dout;
static char max_port[4], max_out_port[4];
static uint32_t nvs_address;

static io_stream_t active_stream;
static embroidery_t api;
static embroidery_settings_t embroidery;
static on_report_options_ptr on_report_options;
static on_state_change_ptr on_state_change;
static on_execute_realtime_ptr on_execute_realtime;
static on_file_open_ptr on_file_open;
static driver_reset_ptr driver_reset;
static limit_interrupt_callback_ptr limits_interrupt_callback;
static embroidery_job_t job = {0};

static bool spindle_control (bool on)
{
    if(job.spindle.on != on) {
        job.spindle.on = on;
        if(embroidery.sync_mode)
            job.plan_data.spindle.hal->set_state(job.plan_data.spindle.hal, job.spindle, job.spindle.on ? 1.0f : 0.0f);
    }

    return on;
}

static void end_job (void)
{
    job.completed = job.enqueued = true;

    if(active_stream.type != StreamType_Null) {
        memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));
        active_stream.type = StreamType_Null;
    }

    if(job.file) {
        vfs_close(job.file);
        job.file = NULL;
    }

    spindle_control(Off);
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static void thread_change (embroidery_thread_color_t color)
{
    const char *thread_color = api.get_thread_color(color);

    spindle_control(Off);

    report_message(thread_color, Message_Info);
    protocol_buffer_synchronize();              // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime();                // Execute suspend.

    job.executed.thread_changes++;
}

static void jump_out (bool on)
{
    if(jump_port == IOPORT_UNASSIGNED)
        hal.coolant.set_state((coolant_state_t){ .mist = on });
    else
        ioport_digital_out(jump_port, on);
}

static void exec_thread_change (void *data)
{
    api.thread_change(job.color);
}

static void thread_trim (void)
{
    spindle_control(Off);

    report_message("trim", Message_Info);
    protocol_buffer_synchronize();              // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime();                // Execute suspend.

    job.executed.trims++;
}

static void exec_thread_trim (void *data)
{
    api.thread_trim();
}

static void exec_hold (void *data)
{
    spindle_control(Off);

    report_message((char *)data, Message_Info);
    protocol_buffer_synchronize();              // Sync and finish all remaining buffered motions before moving on.
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime();                // Execute suspend.
    mc_line(job.position.values, &job.plan_data);
}

static void onStateChanged (sys_state_t state)
{
    static uint32_t last_ms;

    if(job.machine_state == state)
        return;

    switch(state) {

        case STATE_IDLE:
            if(job.await.jump) {
                jump_out(false);
                job.await.jump = false;
            }
            else if(job.stitching && job.machine_state == STATE_CYCLE) {
//                if(job.first)
//                    job.first = false;
 //               else {
                    uint32_t ms = hal.get_elapsed_ticks() - last_ms; //job.last_trigger;
                    job.stitch_interval = max(job.stitch_interval, ms);
                }
//            }
            break;

        case STATE_CYCLE:
            last_ms = hal.get_elapsed_ticks();
            break;
    }

    if(job.machine_state == STATE_HOLD)
        job.await.pause = false;

    job.machine_state = state;

    if(debug_port != IOPORT_UNASSIGNED)
        ioport_digital_out(debug_port, state == STATE_CYCLE);

    if(on_state_change)
        on_state_change(state);
}

static inline void set_needle_trigger (void)
{
    uint32_t ms = hal.get_elapsed_ticks();

    if(job.await.trigger /*&& ms - job.last_trigger > 25*/) {

        job.trigger_interval = ms - job.last_trigger;
        job.trigger_interval_min = min(job.trigger_interval_min, job.trigger_interval);

        if(job.machine_state == STATE_CYCLE) {
            job.errs++;
            return;
        }

        job.await.trigger = false;
    }

    job.executed.stitches++;
    job.last_trigger = ms;
}

ISR_CODE void ISR_FUNC(z_limit_trigger)(limit_signals_t state)
{
    if(state.min.z) {
        state.min.z = Off;
        set_needle_trigger();
    }

    if(limit_signals_merge(state).value)
        limits_interrupt_callback(state);
}

ISR_CODE static void ISR_FUNC(needle_trigger)(uint8_t port, bool state)
{
    set_needle_trigger();
}

ISR_CODE static void ISR_FUNC(thread_break)(uint8_t port, bool state)
{
    if(job.file && !job.await.pause) {
        job.breaks++;
        if(job.executed.stitches > job.breaks + 10)
            task_add_immediate(exec_hold, "Thread break!");
    }
}

static void onExecuteRealtime (sys_state_t state)
{
    static bool busy = false;

    on_execute_realtime(state);

    if(busy || job.completed)
        return;

    if(job.spindle_stop && hal.get_elapsed_ticks() - job.last_trigger >= job.spindle_stop) {
        spindle_control(Off);
        job.spindle_stop = 0;
    }

    if(job.await.value)
        return;

    if(job.enqueued && job.queue.tail == job.queue.head) {

        end_job();
        hal.stream.cancel_read_buffer();

        if(grbl.on_program_completed)
            grbl.on_program_completed(ProgramFlow_CompletedM30, false);

        grbl.report.feedback_message(Message_ProgramEnd);

        return;
    }

    if(plan_get_block_buffer_available() < 3)
        return;

    stitch_t *stitch = &job.queue.stitch[job.queue.tail];

    // Wait for non-stitching moves to complete before starting stitching
    if(!job.stitching && stitch->type == Stitch_Normal && job.machine_state != STATE_IDLE)
        return;

    bool was_stitching = job.stitching;

    busy = true;

    job.queue.tail = ++job.queue.tail & (STITCH_QUEUE_SIZE - 1);

    // If stitching look-ahead to next command to see if we should stop the motor early to avoid overshoot.
    if(job.stitching) {
        if(job.queue.tail != job.queue.head && job.queue.stitch[job.queue.tail].type != Stitch_Normal) {
            if(job.queue.stitch[job.queue.tail].type == Stitch_Jump)
                jump_out(true);
            else if(embroidery.stop_delay)
                job.spindle_stop = embroidery.stop_delay;
        }
    }

    if(!(job.stitching = stitch->type == Stitch_Normal) && embroidery.stop_delay == 0) {
        spindle_control(Off);
        job.spindle_stop = 0;
    }

    switch(stitch->type) {

        case Stitch_Normal:
            job.exced++;
            job.plan_data.condition.rapid_motion = Off;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;

            if((job.first = !job.spindle.on))
                spindle_control(On);

            mc_line(job.position.values, &job.plan_data);

            if(!(job.await.trigger = embroidery.sync_mode)) {
//                plan_data.condition.rapid_motion = On;
                job.position.z = -embroidery.z_travel;
                mc_line(job.position.values, &job.plan_data);
                job.position.z = embroidery.z_travel;
                mc_line(job.position.values, &job.plan_data);
            }
            break;

        case Stitch_Jump:
            job.executed.jumps++;
            job.plan_data.condition.rapid_motion = On;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;

//            if(was_stitching) {
//                job.await.pause = true;
//                task_add_immediate(exec_hold, "Jump");
//            } else
                job.await.jump = mc_line(job.position.values, &job.plan_data);
            break;

        case Stitch_Trim:
            job.await.pause = true;
            job.plan_data.condition.rapid_motion = On;
            job.spindle_stop = embroidery.stop_delay;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;
            mc_line(job.position.values, &job.plan_data);

            task_add_immediate(exec_thread_trim, NULL);
            break;

        case Stitch_Stop:
            job.await.pause = true;
            job.plan_data.condition.rapid_motion = On;

            job.position.x += stitch->target.x;
            job.position.y += stitch->target.y;
//            mc_line(job.position.values, &job.plan_data);

            job.color = stitch->color;
            task_add_immediate(exec_thread_change, NULL);
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
            if(!(job.enqueued = !api.get_stitch(&job.queue.stitch[job.queue.head], job.file))) {

                switch(job.queue.stitch[job.queue.head].type) {
                    case Stitch_Normal:
                        job.programmed.stitches++;
                        break;

                    case Stitch_Jump:
                        job.programmed.jumps++;
                        break;

                    case Stitch_Trim:
                        job.programmed.trims++;
                        break;

                    case Stitch_Stop:
                        job.programmed.thread_changes++;
                        break;

                    case Stitch_SequinEject:
                        job.programmed.sequin_ejects++;
                        break;
                }

                job.queue.head = bptr;
            }
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

    if(request == SpindleData_RPM)
        spindle_data.rpm = job.spindle.on ? 60000.0f / (float)job.trigger_interval : 0.0f;

    return &spindle_data;
}

static status_code_t onFileOpen (const char *fname, vfs_file_t *file, bool stream)
{
    bool ok = false;


    if(brother_open_file(file, &api) || tajima_open_file(file, &api)) {

        if(stream) {

            if(break_port == IOPORT_UNASSIGNED || ioport_wait_on_input(true, break_port, WaitMode_Immediate, 0.0f) == 0) {

                memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Save current stream pointers
                hal.stream.type = StreamType_File;                          // then redirect to read from SD card instead
                hal.stream.read = sdcard_read;                              // ...

                plan_data_init(&job.plan_data);

                job.file = file;
                job.completed = job.enqueued = job.stitching = false;
                job.queue.head = job.queue.tail = job.stitch_interval = job.trigger_interval = job.await.value = job.breaks = 0;
                job.plan_data.feed_rate = embroidery.feedrate;
                job.plan_data.condition.rapid_motion = On;
                if(embroidery.sync_mode)
                    job.plan_data.spindle.hal->get_data = spindleGetData;
                job.plan_data.spindle.hal->cap.at_speed = On,
                system_convert_array_steps_to_mpos(job.position.values, sys.position);

                memset(&job.programmed, 0, sizeof(embroidery_job_details_t));
                memset(&job.executed, 0, sizeof(embroidery_job_details_t));

                job.trigger_interval_min = 10000;
                job.errs = job.exced = 0;
            } else {
                vfs_close(file);
                report_message("No thread detected", Message_Error);
            }
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

static const setting_group_detail_t embroidery_groups [] = {
    { Group_Root, Group_Embroidery, "Embroidery" }
};

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    bool ok = false;

    switch(setting->id) {

        case Setting_UserDefined_2:
        case Setting_UserDefined_5:
        case Setting_UserDefined_7:
            ok = n_din > 0;
            break;

        case Setting_UserDefined_6:
        case Setting_UserDefined_8:
            ok = n_dout > 0;
            break;

        default:
            break;
    }

    return ok;
}

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status;

    if((status = isintf(value) ? Status_OK : Status_BadNumberFormat) == Status_OK)
      switch(setting) {

        case Setting_UserDefined_2:
            embroidery.port = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;
            break;

        case Setting_UserDefined_6:
            embroidery.debug_port = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;
            break;

        case Setting_UserDefined_7:
            embroidery.break_port = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;
            break;

        case Setting_UserDefined_8:
            embroidery.jump_port = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;
            break;

        default: break;
    }

    return status;
}

static float get_port (setting_id_t setting)
{
    float value = -1.0f;

    switch(setting) {

        case Setting_UserDefined_2:
            value = embroidery.port >= n_din ? -1.0f : (float)embroidery.port;
            break;

        case Setting_UserDefined_6:
            value = embroidery.debug_port >= n_dout ? -1.0f : (float)embroidery.debug_port;
            break;

        case Setting_UserDefined_7:
            value = embroidery.break_port >= n_din ? -1.0f : (float)embroidery.break_port;
            break;

        case Setting_UserDefined_8:
            value = embroidery.jump_port >= n_din ? -1.0f : (float)embroidery.jump_port;
            break;

        default: break;
    }

    return value;
}

static const setting_detail_t embroidery_settings[] = {
    { Setting_UserDefined_0, Group_Embroidery, "Embroidery feedrate", "mm/min", Format_Decimal, "####0.0", NULL, NULL, Setting_NonCore, &embroidery.feedrate, NULL, NULL },
    { Setting_UserDefined_1, Group_Embroidery, "Embroidery Z travel", "mm", Format_Decimal, "##0.0", NULL, NULL, Setting_NonCore, &embroidery.z_travel, NULL, NULL },
    { Setting_UserDefined_2, Group_AuxPorts, "Embroidery trigger port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_3, Group_Embroidery, "Embroidery sync mode", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &embroidery.sync_mode, NULL, NULL },
    { Setting_UserDefined_4, Group_Embroidery, "Embroidery stop delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_NonCore, &embroidery.stop_delay, NULL, NULL },
    { Setting_UserDefined_5, Group_Embroidery, "Trigger edge/input", NULL, Format_RadioButtons, "Falling,Rising,Z limit", NULL, NULL, Setting_NonCore, &embroidery.edge, NULL, NULL, { .reboot_required = On } },
    { Setting_UserDefined_6, Group_AuxPorts, "Embroidery debug port", NULL, Format_Decimal, "-#0", "-1", max_out_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_7, Group_AuxPorts, "Thread break port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_UserDefined_8, Group_AuxPorts, "Jump port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t embroidery_settings_descr[] = {
    { Setting_UserDefined_0, "Feedrate to be used when embroidering." },
    { Setting_UserDefined_1, "Z travel per stitch when needle is controlled by a stepper (sync mode = 0)." },
    { Setting_UserDefined_2, "Aux input port to use for needle trigger (sync mode = 1, trigger edge <> Z limit input). Set to -1 to disable." },
    { Setting_UserDefined_3, "When sync mode is enabled XY motion is controlled by needle trigger, else the Z axis stepper runs the needle motor." },
    { Setting_UserDefined_4, "Delay after last needle trigger before stopping needle motor (sync mode = 1)." },
    { Setting_UserDefined_5, "Trigger edge for needle trigger, from aux input or Z limit input (sync mode = 1).\\n\\n"
                             "NOTE: When Z limit input is used hard limits has to be enabled!"
    },
    { Setting_UserDefined_6, "Debug port, outputs high on aux port when XY motion is ongoing. Set to -1 to disable." },
    { Setting_UserDefined_7, "Thread break detection port. Set to -1 to disable." },
    { Setting_UserDefined_8, "Jump output port. Set to -1 to disable." }
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
    embroidery.sync_mode = On;
    embroidery.break_port =
    embroidery.jump_port =
    embroidery.debug_port = IOPORT_UNASSIGNED;
    embroidery.port = ioport_find_free(Port_Digital, Port_Input, (pin_cap_t){ .irq_mode = (embroidery.edge ? IRQ_Mode_Rising : IRQ_Mode_Falling), .claimable = On }, "Embroidery needle trigger");
    embroidery.edge = embroidery.port != IOPORT_UNASSIGNED ? EmbroideryTrig_Falling : EmbroideryTrig_ZLimit;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&embroidery, sizeof(embroidery_settings_t), true);
}

static void embroidery_settings_load (void)
{
    bool ok;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&embroidery, nvs_address, sizeof(embroidery_settings_t), true) != NVS_TransferResult_OK)
        embroidery_settings_restore();

    if(embroidery.port >= n_din)
        embroidery.port = IOPORT_UNASSIGNED;

    if(embroidery.break_port >= n_din)
        embroidery.break_port = IOPORT_UNASSIGNED;

    if(embroidery.debug_port >= n_dout)
        embroidery.debug_port = IOPORT_UNASSIGNED;

    if(embroidery.jump_port >= n_dout)
        embroidery.jump_port = IOPORT_UNASSIGNED;

    if((ok = embroidery.edge == EmbroideryTrig_ZLimit)) {

        hal.driver_cap.software_debounce = Off;

        limits_interrupt_callback = hal.limits.interrupt_callback;
        hal.limits.interrupt_callback = z_limit_trigger;
        // ensure hard limits is enabled - interrupt will not fire if not

    } else if((port = embroidery.port) != IOPORT_UNASSIGNED) {

        xbar_t *portinfo = ioport_get_info(Port_Digital, Port_Input, port);

        if(portinfo && !portinfo->mode.claimed && (portinfo->cap.irq_mode & (embroidery.edge ? IRQ_Mode_Rising : IRQ_Mode_Falling)) && ioport_claim(Port_Digital, Port_Input, &port, "Embroidery needle trigger"))
            ok = ioport_enable_irq(port, embroidery.edge ? IRQ_Mode_Rising : IRQ_Mode_Falling, needle_trigger);
    }

    if(ok) {
        if((break_port = embroidery.break_port) != IOPORT_UNASSIGNED) {

            xbar_t *portinfo = ioport_get_info(Port_Digital, Port_Input, break_port);

            if(portinfo && !portinfo->mode.claimed && (portinfo->cap.irq_mode & (portinfo->mode.inverted ? IRQ_Mode_Rising : IRQ_Mode_Falling)) && ioport_claim(Port_Digital, Port_Input, &break_port, "Embroidery thread break"))
                ioport_enable_irq(break_port, portinfo->mode.inverted ? IRQ_Mode_Rising : IRQ_Mode_Falling, thread_break);
            else {
                break_port = IOPORT_UNASSIGNED;
                task_run_on_startup(report_warning, "Embroidery plugin failed to claim port for thread break detection!");
            }
        }

        if((jump_port = embroidery.jump_port) != IOPORT_UNASSIGNED) {
            if(!ioport_claim(Port_Digital, Port_Output, &jump_port, "Embroidery jump output"))
                jump_port = IOPORT_UNASSIGNED;
        }

        if((debug_port = embroidery.debug_port) != IOPORT_UNASSIGNED) {
            if(!ioport_claim(Port_Digital, Port_Output, &debug_port, "Embroidery debug output"))
                debug_port = IOPORT_UNASSIGNED;
        }
    } else
        task_run_on_startup(report_warning, "Embroidery plugin failed to initialize, no pin for needle trigger signal!");
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("EMBROIDERY", "0.12");
}

const char *embroidery_get_thread_color (embroidery_thread_color_t color)
{
    return api.get_thread_color ? api.get_thread_color(color) : NULL;
}

void embroidery_set_thread_trim_handler (thread_trim_ptr handler)
{
    api.thread_trim = handler;
}

void embroidery_set_thread_change_handler (thread_change_ptr handler)
{
    api.thread_change = handler;
}

void embroidery_init (void)
{
    static setting_details_t setting_details = {
        .groups = embroidery_groups,
        .n_groups = sizeof(embroidery_groups) / sizeof(setting_group_detail_t),
        .settings = embroidery_settings,
        .n_settings = sizeof(embroidery_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = embroidery_settings_descr,
        .n_descriptions = sizeof(embroidery_settings_descr) / sizeof(setting_descr_t),
    #endif
        .load = embroidery_settings_load,
        .restore = embroidery_settings_restore,
        .save = embroidery_settings_save
    };

    if((nvs_address = nvs_alloc(sizeof(embroidery_settings_t)))) {

        job.completed = true;
        active_stream.type = StreamType_Null;

        n_din = ioports_available(Port_Digital, Port_Input);
        strcpy(max_port, uitoa(n_din - 1));

        n_dout = ioports_available(Port_Digital, Port_Output);
        strcpy(max_out_port, uitoa(n_dout - 1));

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

        api.thread_trim = thread_trim;
        api.thread_change = thread_change;
    } else
        task_run_on_startup(report_warning, "Embroidery plugin failed to initialize, no NVS storage for settings!");
}

#endif // EMBROIDERY_ENABLE
