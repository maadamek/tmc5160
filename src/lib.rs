//! A platform agnostic driver to interface with the TMC5160 (Trinamic integrated stepper motor controller)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2
//!
#![no_std]
#![allow(dead_code)]
#![deny(missing_docs)]
#![deny(warnings)]

use core::fmt;
use core::result::Result;
use embedded_hal_async::spi::{Mode, Phase, Polarity, SpiDevice};

use crate::registers::*;

pub mod registers;

fn swap_bytes(input: [u8; 4]) -> [u8; 4] {
    let mut output = [0; 4];
    for i in 0..4 {
        output[4 - 1 - i] = input[i];
    }
    output
}

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

/// Data Exchange packet
pub struct DataPacket {
    /// Status returned from last communication
    pub status: SpiStatus,
    /// Data received from TMC5160
    pub data: u32,
    /// debug
    pub debug: [u8; 5],
}

impl fmt::Display for DataPacket {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "0x{:x}:0x{:x}", self.status.into_bytes()[0], self.data)
    }
}

/// TMC5160 driver
pub struct Tmc5160<SPI> {
    spi: SPI,
    /// the max velocity that is set
    pub v_max: f32,
    /// status register of the driver
    pub status: SpiStatus,
    /// debug info of the last transmission
    pub debug: [u8; 5],
    _clock: f32,
    _step_count: f32,
    /// value of the GCONF register
    pub g_conf: GConf,
    /// value of the NODECONF register
    pub node_conf: NodeConf,
    /// value of the OTPPROG register
    pub otp_prog: OtpProg,
    /// value of the SHORT_CONF register
    pub short_conf: ShortConf,
    /// value of the DRV_CONF register
    pub drv_conf: DrvConf,
    /// value of the IHOLD_IRUN register
    pub ihold_irun: IHoldIRun,
    /// value of the SWMODE register
    pub sw_mode: SwMode,
    /// value of the ENCMODE register
    pub enc_mode: EncMode,
    /// value of the MSLUTSEL register
    pub ms_lut_sel: MsLutSel,
    /// value of the CHOPCONF register
    pub chop_conf: ChopConf,
    /// value of the COOLCONF register
    pub cool_conf: CoolConf,
    /// value of the PWMCONF register
    pub pwm_conf: PwmConf,
}

impl<SPI> Tmc5160<SPI>
where
    SPI: SpiDevice<u8>,
{
    /// Create a new driver from an SpiDevice
    pub fn new(spi: SPI) -> Self {
        Tmc5160 {
            spi,
            v_max: 0.0,
            status: SpiStatus::new(),
            debug: [0; 5],
            _clock: 12000000.0,
            _step_count: 256.0,
            g_conf: GConf::new(),
            node_conf: NodeConf::new(),
            otp_prog: OtpProg::new(),
            short_conf: ShortConf::new(),
            drv_conf: DrvConf::new(),
            ihold_irun: IHoldIRun::new(),
            sw_mode: SwMode::new(),
            enc_mode: EncMode::new(),
            ms_lut_sel: MsLutSel::new(),
            chop_conf: ChopConf::new(),
            cool_conf: CoolConf::new(),
            pwm_conf: PwmConf::new(),
        }
    }

    /// specify clock speed of the Tmc5160 (Default is 12 MHz)
    pub fn clock(mut self, clock: f32) -> Self {
        self._clock = clock;
        self
    }

    /// specify step count of the motor (Default is 256)
    pub fn step_count(mut self, step_count: f32) -> Self {
        self._step_count = step_count;
        self
    }

    fn speed_from_hz(&mut self, speed_hz: f32) -> u32 {
        (speed_hz / (self._clock / 16_777_216.0) * self._step_count) as u32
    }

    fn accel_from_hz(&mut self, accel_hz_per_s: f32) -> u32 {
        (accel_hz_per_s / (self._clock * self._clock)
            * (512.0 * 256.0)
            * 16_777_216.0
            * self._step_count) as u32
    }

    /// read a specified register
    pub async fn read_register<T>(&mut self, reg: T) -> Result<DataPacket, SPI::Error>
    where
        T: Address + Copy,
    {
        // Process cmd to read, return previous (dummy) state
        let _dummy = self.read_io(reg).await?;
        // Repeat cmd to read, return state
        self.read_io(reg).await
    }

    async fn read_io<T>(&mut self, reg: T) -> Result<DataPacket, SPI::Error>
    where
        T: Address + Copy,
    {
        let buffer = [reg.addr(), 0, 0, 0, 0];

        let mut response_buffer = [0u8; 5];

        self.spi.transfer(&mut response_buffer, &buffer).await?;

        let mut ret_val: [u8; 4] = [0; 4];

        ret_val.copy_from_slice(&response_buffer[1..(4 + 1)]);

        let mut debug_val: [u8; 5] = [0; 5];

        debug_val.copy_from_slice(&response_buffer);

        Ok(DataPacket {
            status: SpiStatus::from_bytes([response_buffer[0]]),
            data: u32::from_be_bytes(ret_val),
            debug: debug_val,
        })
    }

    /// write value to a specified register
    pub async fn write_register<T>(
        &mut self,
        reg: T,
        val: &mut [u8; 4],
    ) -> Result<DataPacket, SPI::Error>
    where
        T: Address + Copy,
    {
        let buffer = [reg.addr() | 0x80, val[0], val[1], val[2], val[3]];

        let debug_val = buffer.clone();

        let mut response_buffer = [0u8; 5];

        self.spi.transfer(&mut response_buffer, &buffer).await?;

        let mut ret_val: [u8; 4] = [0; 4];

        ret_val.copy_from_slice(&response_buffer[1..(4 + 1)]);

        Ok(DataPacket {
            status: SpiStatus::from_bytes([response_buffer[0]]),
            data: u32::from_be_bytes(ret_val),
            debug: debug_val,
        })
    }

    fn mres_to_microsteps(&self) -> u16 {
        match self.chop_conf.mres() {
            0 => 256,
            x => (2_u16).pow(x as u32),
        }
    }

    /// clear G_STAT register
    pub async fn clear_g_stat(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = 0b111_u32.to_be_bytes();
        //let mut value= (!0_u32).to_be_bytes();
        self.write_register(Registers::GSTAT, &mut value).await
    }

    /// sets the maximum allowed encoder deviation
    /// if value is 0, deviation warnings are disabled
    pub async fn set_max_enc_deviation(
        &mut self,
        max_deviation: u32,
    ) -> Result<DataPacket, SPI::Error> {
        let mut value = max_deviation.to_be_bytes();
        self.write_register(Registers::ENC_DEVIATION, &mut value)
            .await
    }

    /// Sets the current encoder position to 0
    pub async fn set_enc_home(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = [0; 4];
        self.write_register(Registers::X_ENC, &mut value).await
    }

    /// sets the encoder resolution
    /// the calculation depends on the microstep resolution and the ENCMode.enc_sel_decimal
    /// if any of the these registers change set_enc_resolution needs to be configured again!
    pub async fn set_enc_resolution(&mut self, encoder_cpr: i32) -> Result<DataPacket, SPI::Error> {
        let multiplier = if self.enc_mode.enc_sel_decimal() {
            10e4
        } else {
            2e16
        };
        let enc_const = encoder_cpr as f32 / self.mres_to_microsteps() as f32;

        let factor = enc_const as i32;
        let fraction = ((enc_const - factor as f32) * multiplier) as i32;
        let mut value = ((factor << 16) | fraction).to_be_bytes();
        self.write_register(Registers::ENC_CONST, &mut value).await
    }

    /// get ENC_STATUS register
    pub async fn get_enc_status(&mut self) -> Result<EncStatus, SPI::Error> {
        self.read_register(Registers::ENC_STATUS)
            .await
            .map(|res| EncStatus::from_bytes(res.data.to_le_bytes()))
    }

    /// get current encoder position
    pub async fn get_enc_position(&mut self) -> Result<i32, SPI::Error> {
        self.read_register(Registers::X_ENC)
            .await
            .map(|res| i32::from_le_bytes(res.data.to_le_bytes()))
    }

    /// clear ENC_STATUS register
    pub async fn clear_enc_status(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = 0b111_u32.to_be_bytes();
        //let mut value= (!0_u32).to_be_bytes();
        self.write_register(Registers::ENC_STATUS, &mut value).await
    }

    /// write value to SW_MODE register
    pub async fn update_sw_mode(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.sw_mode.into_bytes());
        self.write_register(Registers::SW_MODE, &mut value).await
    }

    /// write value to G_CONF register
    pub async fn update_g_conf(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.g_conf.into_bytes());
        self.write_register(Registers::GCONF, &mut value).await
    }

    /// write value to CHOP_CONF register
    pub async fn update_chop_conf(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.chop_conf.into_bytes());
        self.write_register(Registers::CHOPCONF, &mut value).await
    }

    /// write value to COOL_CONF register
    pub async fn update_cool_conf(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.cool_conf.into_bytes());
        self.write_register(Registers::COOLCONF, &mut value).await
    }

    /// write value to IHOLD_IRUN register
    pub async fn update_ihold_irun(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.ihold_irun.into_bytes());
        self.write_register(Registers::IHOLD_IRUN, &mut value).await
    }

    /// write value to PWM_CONF register
    pub async fn update_pwm_conf(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.pwm_conf.into_bytes());
        self.write_register(Registers::PWMCONF, &mut value).await
    }

    /// write value to ENC_MODE register
    pub async fn update_enc_mode(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut value = swap_bytes(self.enc_mode.into_bytes());
        self.write_register(Registers::ENCMODE, &mut value).await
    }

    /// write value to GLOBALSCALER register
    pub async fn set_global_scaler(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::GLOBALSCALER, &mut value)
            .await
    }

    /// write value to TPOWERDOWN register
    pub async fn set_tpowerdown(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TPOWERDOWN, &mut value).await
    }

    /// write value to TPWMTHRS register
    pub async fn set_tpwmthrs(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TPWMTHRS, &mut value).await
    }

    /// write value to TCOOLTHRS register
    pub async fn set_tcoolthrs(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::TCOOLTHRS, &mut value).await
    }

    /// write value to A1 register
    pub async fn set_a1(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::A1, &mut value).await
    }

    /// write value to V1 register
    pub async fn set_v1(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::V1, &mut value).await
    }

    /// write value to AMAX register
    pub async fn set_amax(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::AMAX, &mut value).await
    }

    /// write value to VMAX register
    pub async fn set_vmax(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VMAX, &mut value).await
    }

    /// write value to DMAX register
    pub async fn set_dmax(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::DMAX, &mut value).await
    }

    /// write value to D1 register
    pub async fn set_d1(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::D1, &mut value).await
    }

    /// write value to VSTART register
    pub async fn set_vstart(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VSTART, &mut value).await
    }

    /// write value to VSTOP register
    pub async fn set_vstop(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::VSTOP, &mut value).await
    }

    /// write value to PWM_AUTO register
    pub async fn set_pwm_auto(&mut self, val: u32) -> Result<DataPacket, SPI::Error> {
        let mut value = val.to_be_bytes();
        self.write_register(Registers::PWM_AUTO, &mut value).await
    }

    /// write value to RAMPMODE register
    pub async fn set_rampmode(&mut self, val: RampMode) -> Result<DataPacket, SPI::Error> {
        let mut value = (val as u32).to_be_bytes();
        self.write_register(Registers::RAMPMODE, &mut value).await
    }

    /// read offset register
    pub async fn read_offset(&mut self) -> Result<u32, SPI::Error> {
        self.read_register(Registers::OFFSET_READ)
            .await
            .map(|packet| packet.data)
    }

    /// read TSTEP register
    pub async fn read_tstep(&mut self) -> Result<u32, SPI::Error> {
        self.read_register(Registers::TSTEP)
            .await
            .map(|packet| packet.data)
    }

    /// read DRV_STATUS register
    pub async fn read_drv_status(&mut self) -> Result<DrvStatus, SPI::Error> {
        let packet = self.read_register(Registers::DRV_STATUS).await?;
        self.status = packet.status;
        Ok(DrvStatus::from_bytes(packet.data.to_le_bytes()))
    }

    /// read GSTAT register
    pub async fn read_gstat(&mut self) -> Result<GStat, SPI::Error> {
        let packet = self.read_register(Registers::GSTAT).await?;
        self.status = packet.status;
        self.debug = packet.debug;
        Ok(GStat::from_bytes(packet.data.to_le_bytes()))
    }

    /// read GCONF register
    pub async fn read_gconf(&mut self) -> Result<GConf, SPI::Error> {
        let packet = self.read_register(Registers::GCONF).await?;
        self.status = packet.status;
        Ok(GConf::from_bytes(packet.data.to_le_bytes()))
    }

    /// read RAMP_STAT register
    pub async fn read_ramp_status(&mut self) -> Result<RampStat, SPI::Error> {
        let packet = self.read_register(Registers::RAMP_STAT).await?;
        self.status = packet.status;
        Ok(RampStat::from_bytes(packet.data.to_le_bytes()))
    }

    /// read ENC_STATUS register
    pub async fn read_enc_status(&mut self) -> Result<EncStatus, SPI::Error> {
        let packet = self.read_register(Registers::ENC_STATUS).await?;
        self.status = packet.status;
        Ok(EncStatus::from_bytes(packet.data.to_le_bytes()))
    }

    /// set the position to 0 / home
    pub async fn set_home(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut val = 0_u32.to_be_bytes();
        self.write_register(Registers::XACTUAL, &mut val).await?;
        let packet = self.write_register(Registers::XTARGET, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// stop the motor now
    pub async fn stop(&mut self) -> Result<DataPacket, SPI::Error> {
        let mut val = 0_u32.to_be_bytes();
        self.write_register(Registers::VSTART, &mut val).await?;
        self.write_register(Registers::VMAX, &mut val).await?;
        // TODO: check how we can restart the movement afterwards
        let mut position = self.get_position().await?.to_be_bytes();
        let packet = self
            .write_register(Registers::XTARGET, &mut position)
            .await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// check if the motor is moving
    pub async fn is_moving(&mut self) -> Result<bool, SPI::Error> {
        self.read_drv_status()
            .await
            .map(|packet| !packet.standstill())
    }

    /// check if the motor has reached the target position
    pub async fn position_is_reached(&mut self) -> Result<bool, SPI::Error> {
        self.read_ramp_status()
            .await
            .map(|packet| packet.position_reached())
    }

    /// check if the motor has reached the constant velocity
    pub async fn velocity_is_reached(&mut self) -> Result<bool, SPI::Error> {
        self.read_ramp_status()
            .await
            .map(|packet| packet.velocity_reached())
    }

    /// check if motor is at right limit
    pub async fn is_at_limit_r(&mut self) -> Result<bool, SPI::Error> {
        self.read_ramp_status()
            .await
            .map(|packet| packet.status_stop_r())
    }

    /// check if motor is at left limit
    pub async fn is_at_limit_l(&mut self) -> Result<bool, SPI::Error> {
        self.read_ramp_status()
            .await
            .map(|packet| packet.status_stop_l())
    }

    /// set the max velocity (VMAX)
    pub async fn set_velocity(&mut self, velocity: f32) -> Result<DataPacket, SPI::Error> {
        self.v_max = velocity;
        let v_max = self.speed_from_hz(velocity);
        let mut val = v_max.to_be_bytes();
        let packet = self.write_register(Registers::VMAX, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// set the max velocity (VMAX)
    pub async fn set_velocity_raw(&mut self, velocity: u32) -> Result<DataPacket, SPI::Error> {
        self.v_max = velocity as f32 / self._step_count * (self._clock / 16_777_216.0);
        let mut val = velocity.to_be_bytes();
        let packet = self.write_register(Registers::VMAX, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// set the max acceleration (AMAX, DMAX, A1, D1)
    pub async fn set_acceleration(&mut self, acceleration: f32) -> Result<DataPacket, SPI::Error> {
        let a_max = self.accel_from_hz(acceleration);
        let mut val = a_max.to_be_bytes();
        self.write_register(Registers::AMAX, &mut val).await?;
        self.write_register(Registers::DMAX, &mut val).await?;
        self.write_register(Registers::A1, &mut val).await?;
        let packet = self.write_register(Registers::D1, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// move to a specific location
    pub async fn move_to(&mut self, target: f32) -> Result<DataPacket, SPI::Error> {
        let target = (target * self._step_count) as i32;
        let mut val = target.to_be_bytes();
        let packet = self.write_register(Registers::XTARGET, &mut val).await?;
        self.status = packet.status;
        Ok(packet)
    }

    /// get the latched position
    pub async fn get_latched_position(&mut self) -> Result<f32, SPI::Error> {
        self.read_register(Registers::XLATCH)
            .await
            .map(|val| (val.data as i32) as f32 / self._step_count)
    }

    /// get the current position
    pub async fn get_position(&mut self) -> Result<f32, SPI::Error> {
        self.read_register(Registers::XACTUAL)
            .await
            .map(|val| (val.data as i32) as f32 / self._step_count)
    }

    /// set the current position
    pub async fn set_position(&mut self, target_signed: i32) -> Result<DataPacket, SPI::Error> {
        let target = target_signed;
        let mut val = (target * self._step_count as i32).to_be_bytes();
        self.write_register(Registers::XACTUAL, &mut val).await
    }

    /// get the current velocity
    pub async fn get_velocity(&mut self) -> Result<f32, SPI::Error> {
        self.read_register(Registers::VACTUAL).await.map(|target| {
            if (target.data & 0b100000000000000000000000) == 0b100000000000000000000000 {
                ((16777216 - target.data as i32) as f64 / self._step_count as f64) as f32
            } else {
                ((target.data as i32) as f64 / self._step_count as f64) as f32
            }
        })
    }

    /// get the set maximum velocity (VMAX)
    pub fn get_velocity_max(&mut self) -> f32 {
        self.v_max
    }

    /// get the current target position (XTARGET)
    pub async fn get_target(&mut self) -> Result<f32, SPI::Error> {
        self.read_register(Registers::XTARGET)
            .await
            .map(|packet| packet.data as f32 / self._step_count)
    }
}
