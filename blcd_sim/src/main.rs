extern crate rand;

use rand::Rng;

use std::f32;

use crate::common::*;
extern crate gnuplot;
use gnuplot::*;

mod common;

struct PIDController {
	Kp: f32,  /* Proportional */
	Ki: f32,  /* Integral */
	Kd: f32,  /* Derivative */
	setpoint: f32,
	dt: f32,  /* sampling period */
	intgr: f32,  /* integration */
	deriv: f32,  /* derivative */
	err_prev: f32,  /* previous error */
}

impl PIDController {
	fn control(&mut self, meas: f32) -> f32 {
		let mut err = self.setpoint - meas;
		self.intgr = self.intgr + err * self.dt;
		self.deriv = (err - self.err_prev) / self.dt;
		let mut output = self.Kp * err + self.Ki * self.intgr +
		self.Kd*self.deriv;
		self.err_prev = err;
		output
	}
}

/* See https://en.wikipedia.org/wiki/Low-pass_filter */
struct LPFilter {
	alpha: f32,
	current: f32
}

impl LPFilter {
	fn filter(&mut self, val: f32) -> f32 {
		let newval = self.current + self.alpha * (val - self.current);
		self.current = newval;
		newval
	}
}

struct TempModel {
	Tc: f32,  /* Time constant */
	Kp: f32,  /* Factor */
	dt: f32,  /* sampling period */
	Ti: f32,  /* current temp */
	Tp: f32,  /* constant time factor */
	dist: f32,  /* disturbance */
}

impl TempModel {
	fn init_Tp(&mut self) {
		self.Tp = 1. / (self.Tc / self.dt + 1.);
	}

	fn next_temp(&mut self, input: f32) -> f32 {
		let Ti = self.Ti + self.Tp * (self.Kp * input + self.dist - self.Ti);
		self.Ti = Ti;
		Ti
	}
}

fn plot2d(c: Common, time: &Vec<f32>, temp: &Vec<f32>)
{
	let mut fg = Figure::new();
	c.set_term(&mut fg);

	fg.axes2d()
		.set_title("Temperature", &[])
		.set_legend(Graph(0.5), Graph(0.9), &[], &[])
		.set_x_label("Time", &[])
		.set_y_label("Temperature", &[])
		.lines(
			time,
			temp,
			&[Caption("Temp.")],
		);

	c.show(&mut fg, "fg.temperature.gnuplot");
	if !c.no_show
	{
		fg.set_terminal("pngcairo", "fg.temperature.png");
		fg.show();
	}
}

fn temp_model(setpoint: f32, outtemp: f32, dt: f32, n: usize) -> (Vec<f32>, Vec<f32>)
{
    let mut noise_ = rand::thread_rng();
    let mut time = Vec::new();
    let mut temp = Vec::new();
	let mut T_model = TempModel {Tc: 20., Kp: 1., dt: dt, Ti: 0., Tp: 0., dist: outtemp};
	let mut pid = PIDController {Kp: 1., Ki: 0.04, Kd: 0., setpoint: setpoint, dt: dt,
	intgr: 0., deriv: 0., err_prev: 0.};
	T_model.init_Tp();
    time.push(0.);
    temp.push(0.);
    for i in 1..n {
        let ti_: f32 = time[i-1];
        time.push(dt + ti_);
        let ti: f32 = time[i];
        let noise: f32 = noise_.gen::<f32>()/1.;
        //temp.push(1. - 1./ti.exp() + noise);
		let ctl = pid.control(temp[i-1]);
		temp.push(T_model.next_temp(ctl) + noise);
    }
    (time, temp)
}

fn main()
{
    const SIZE: usize = 10000;
    let (time, temp) = temp_model(40., 25., 0.01, SIZE);
	let mut tempf = Vec::new();
	let mut lpf = LPFilter {alpha: 0.05, current: temp[0],};
	tempf.push(temp[0]);
	for i in 1..SIZE {
		tempf.push(lpf.filter(temp[i]));
	}
	
	Common::new().map(|c| plot2d(c, &time, &tempf));
}

fn plot2d_array(c: Common, time: &[f32], temp: &[f32])
{
	let mut fg = Figure::new();
	c.set_term(&mut fg);

	fg.axes2d()
		.set_title("Temperature", &[])
		.set_legend(Graph(0.5), Graph(0.9), &[], &[])
		.set_x_label("Time", &[])
		.set_y_label("Temperature", &[])
		.lines(
			time,
			temp,
			&[Caption("Temp.")],
		);

	c.show(&mut fg, "fg.temperature.gnuplot");
	if !c.no_show
	{
		fg.set_terminal("pngcairo", "fg.temperature.png");
		fg.show();
	}
}

fn main_array()
{
    const SIZE: usize = 7;
    let mut time: [f32; SIZE] = [0.; 7];
    for i in 1..time.len() {
        time[i] = 0.1 + time[i-1];
    }
	Common::new().map(|c| plot2d_array(c, &time, &time));
}