/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2018 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: core_svc.h 592 2018-12-08 01:59:36Z ertl-hiro $
 */

#pragma once

#include <cstdint>
#include <cassert>

namespace ev3svc {
  using bool_t = int;

  // The pixels are packed in the format of ST7586's DDRAM.
  struct bitmap_t {
    int32_t width;
    int32_t height;
    void*   pixels;
  };

  enum brickbtn_t {
    BRICK_BUTTON_LEFT  = 0,
    BRICK_BUTTON_RIGHT = 1,
    BRICK_BUTTON_UP    = 2,
    BRICK_BUTTON_DOWN  = 3,
    BRICK_BUTTON_ENTER = 4,
    BRICK_BUTTON_BACK  = 5,
    TNUM_BRICK_BUTTON  = 6, //!< Number of buttons
  };

  const int MAX_DEVICE_DATALENGTH = 32; //!< Max device data length, derived from MAX_DEVICE_DATALENGTH in 'lms2012.h'
  const int TNUM_INPUT_PORT = 4;  //!< Number of input ports in the system, derived from INPUTS in 'lms2012.h'
  const int TNUM_OUTPUT_PORT = 4;  //!< Number of output ports in the system, derived from OUTPUTS in 'lms2012.h'

  /**
   * ADC
   */

  const int ADC_REF = 5000; //!< [mV]  maximal value on ADC, derived from ADC_REF in 'lms2012.h'
  const int ADC_RES = 4095; //!< [CNT] maximal count on ADC, derived from ADC_RES in 'lms2012.h'

  static inline
  int adc_count_to_mv(int count) {
    return count * ADC_REF / ADC_RES;
  }


  /**
   * Battery info from ADC values
   */

  const float AMP_CIN  = 22.0f; // Constant from 'c_ui.c'
  const float SHUNT_IN = 0.11f; // Constant from 'c_ui.c'
  const float VCE      = 0.05f; // Constant from 'c_ui.c'
  const float AMP_VIN  = 0.5f;  // Constant from 'c_ui.c'

  static inline int adc_count_to_battery_current_mA(int count) {
    return adc_count_to_mv(count) / (AMP_CIN * SHUNT_IN);
  }

  static inline int adc_count_to_battery_voltage_mV(int current_count, int voltage_count) {
    int C_in_mV = adc_count_to_mv(current_count) / AMP_CIN;
    return adc_count_to_mv(voltage_count) / AMP_VIN + C_in_mV + VCE * 1000;
  }

  struct uart_data_t {
    volatile int8_t   (*raw)[MAX_DEVICE_DATALENGTH]; //!< A pointer to raw values from UART sensor
    volatile uint16_t *actual;                       //!< Current raw value is raw[*actual]
    volatile int8_t   *status;                       //!< Bitmap of UART sensor's status
  };

  struct analog_data_t {
    volatile int16_t  *pin1;   //!< Raw value from analog device
    volatile int16_t  *pin6;   //!< Raw value from analog device
    volatile uint16_t *actual; //!< Current raw value is pin1[*actual], pin6[*actual]
  };

  struct i2c_data_t {
    volatile uint8_t  *raw;    //!< Raw value from I2C sensor
    volatile uint8_t  *status; //!< Status of I2C sensor
  };
  const int I2C_TRANS_IDLE = 0;

  struct motor_data_t {
    volatile int8_t  *speed;       //!< Speed, range from -100 to +100
    volatile int32_t *tachoSensor; //!< Angular position (rotary encoder)
  };

  struct font_t {
    // Font information
    uint32_t height;
    uint32_t width; // 0 if not monospace
    // Glyph array for fast seeking (e.g. ASCII printable characters)
    uint32_t  first_code_point; // UTF-8 code point of the first char in the array
    bitmap_t *array;
    uint32_t  array_sz;
    // Glyph dictionary for more characters (e.g. CJK), not supported yet
    void     *dict;
    uint32_t  dict_sz;
  };

  struct brickinfo_t {
    uart_data_t     *uart_sensors;   //!< Pointer of an array with type uart_data_t[TNUM_INPUT_PORT]
    analog_data_t   *analog_sensors; //!< Pointer of an array with type analog_data_t[TNUM_INPUT_PORT]
    i2c_data_t      *i2c_sensors; //!< Pointer of an array with type i2c_data_t[TNUM_INPUT_PORT]
    motor_data_t    *motor_data;     //!< Pointer of an array with type motor_data_t[TNUM_OUTPUT_PORT]
    uint8_t         *motor_ready;    //!< Pointer of a bitmap with type uint8_t
    volatile bool_t *button_pressed; //!< Pointer of an array with type bool_t[TNUM_BRICK_BUTTON]
    bitmap_t        *lcd_screen;     //!< Pointer of a bitmap_t
    font_t          *font_w6h8;      //!< Pointer of a font_t
    font_t          *font_w10h16;    //!< Pointer of a font_t
    void            *app_heap;
    /* Battery */
    int16_t         *motor_current;   //<! Current flowing to motors [ADC count]
    int16_t         *battery_current; //<! Current flowing from the battery [ADC count]
    int16_t         *battery_voltage; //<! Battery voltage [ADC count]
    int16_t         *battery_temp;    //<! Battery temperature [ADC count]
  };

  namespace svc {
    /// @brief calls SVC with up to 5 parameters assigned to r0-r4
    /// @param service_id the service ID to be called
    /// @param par1 the value to be assigned to r0
    /// @param par2 the value to be assigned to r1
    /// @param par3 the value to be assigned to r2
    /// @param par4 the value to be assigned to r3
    /// @param par5 the value to be assigned to r4
    /// @return the value returned by the called service
    inline uint32_t call_svc(intptr_t service_id, intptr_t par1, intptr_t par2, intptr_t par3, intptr_t par4, intptr_t par5) {
      register intptr_t r0 asm("r0") = par1;
      register intptr_t r1 asm("r1") = par2;
      register intptr_t r2 asm("r2") = par3;
      register intptr_t r3 asm("r3") = par4;
      register intptr_t r4 asm("r4") = par5;
      register intptr_t r5 asm("r5") = service_id;

      __asm__ volatile("svc %5"
        : "=r"(r0), "=r"(r1), "=r"(r2), "=r"(r3), "=r"(r4)
        : "I"(1), "0"(r0), "1"(r1), "2"(r2), "3"(r3), "4"(r4), "r"(r5)
        : "r12", "lr", "memory", "cc");
      
        return ((uint32_t) r0);
    }

    /// @brief calls motor SVC
    /// @param cmd pointer to the command structure
    /// @param size size of the command structure
    /// @return the value returned by the called service
    inline uint32_t call_motor_svc(const void* cmd, uint32_t size) {
      return call_svc(23, (intptr_t)cmd, (intptr_t)size, 0, 0, 0);
    }

    /// @brief fetches the brick information to call brick SVC
    /// @param p_brickinfo pointer to the brickinfo_t structure to store the information
    /// @return error code
    inline uint32_t fetch_brick_info(ev3svc::brickinfo_t *p_brickinfo) {
      int32_t ercd = call_svc(24 /*TFN_FETCH_BRICK_INFO*/, (intptr_t)p_brickinfo, (intptr_t)0, (intptr_t)0, 0, 0);
      assert(ercd != -33 /* E_NOMEM */);
      return ercd;
    }
  }
}