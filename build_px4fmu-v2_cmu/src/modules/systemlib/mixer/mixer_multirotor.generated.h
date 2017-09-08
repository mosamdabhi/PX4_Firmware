/*
* This file is automatically generated by multi_tables - do not edit.
*/

#ifndef _MIXER_MULTI_TABLES
#define _MIXER_MULTI_TABLES

enum class MultirotorGeometry : MultirotorGeometryUnderlyingType {
	QUAD_X,
	QUAD_H,
	QUAD_PLUS,
	QUAD_V,
	QUAD_WIDE,
	QUAD_DANAUS,
	QUAD_DEADCAT,
	HEX_X,
	HEX_PLUS,
	HEX_COX,
	OCTA_X,
	OCTA_PLUS,
	OCTA_COX,
	TWIN_ENGINE,
	TRI_Y,

	MAX_GEOMETRY
}; // enum class MultirotorGeometry

namespace {
const MultirotorMixer::Rotor _config_quad_x[] = {
	{ -0.707107,  0.707107,  1.000000,  1.000000 },
	{  0.707107, -0.707107,  1.000000,  1.000000 },
	{  0.707107,  0.707107, -1.000000,  1.000000 },
	{ -0.707107, -0.707107, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_h[] = {
	{ -0.707107,  0.707107, -1.000000,  1.000000 },
	{  0.707107, -0.707107, -1.000000,  1.000000 },
	{  0.707107,  0.707107,  1.000000,  1.000000 },
	{ -0.707107, -0.707107,  1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_plus[] = {
	{ -1.000000,  0.000000,  1.000000,  1.000000 },
	{  1.000000,  0.000000,  1.000000,  1.000000 },
	{  0.000000,  1.000000, -1.000000,  1.000000 },
	{ -0.000000, -1.000000, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_v[] = {
	{ -0.322266,  0.946649,  0.424200,  1.000000 },
	{  0.322266,  0.946649,  1.000000,  1.000000 },
	{  0.322266,  0.946649, -0.424200,  1.000000 },
	{ -0.322266,  0.946649, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_wide[] = {
	{ -0.927184,  0.374607,  1.000000,  1.000000 },
	{  0.777146, -0.629320,  1.000000,  1.000000 },
	{  0.927184,  0.374607, -1.000000,  1.000000 },
	{ -0.777146, -0.629320, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_danaus[] = {
	{ -0.798636,  0.601815,  1.000000,  1.000000 },
	{  0.798636, -0.601815,  1.000000,  1.000000 },
	{  0.798636,  0.601815, -1.000000,  1.000000 },
	{ -0.798636, -0.601815, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_quad_deadcat[] = {
	{ -0.891007,  0.453990,  1.000000,  1.000000 },
	{  0.707107, -0.707107,  1.000000,  0.964000 },
	{  0.891007,  0.453990, -1.000000,  1.000000 },
	{ -0.707107, -0.707107, -1.000000,  0.964000 },
};

const MultirotorMixer::Rotor _config_hex_x[] = {
	{ -1.000000,  0.000000, -1.000000,  1.000000 },
	{  1.000000,  0.000000,  1.000000,  1.000000 },
	{  0.500000,  0.866025, -1.000000,  1.000000 },
	{ -0.500000, -0.866025,  1.000000,  1.000000 },
	{ -0.500000,  0.866025,  1.000000,  1.000000 },
	{  0.500000, -0.866025, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_hex_plus[] = {
	{  0.000000,  1.000000, -1.000000,  1.000000 },
	{ -0.000000, -1.000000,  1.000000,  1.000000 },
	{  0.866025, -0.500000, -1.000000,  1.000000 },
	{ -0.866025,  0.500000,  1.000000,  1.000000 },
	{  0.866025,  0.500000,  1.000000,  1.000000 },
	{ -0.866025, -0.500000, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_hex_cox[] = {
	{ -0.866025,  0.500000, -1.000000,  1.000000 },
	{ -0.866025,  0.500000,  1.000000,  1.000000 },
	{ -0.000000, -1.000000, -1.000000,  1.000000 },
	{ -0.000000, -1.000000,  1.000000,  1.000000 },
	{  0.866025,  0.500000, -1.000000,  1.000000 },
	{  0.866025,  0.500000,  1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_octa_x[] = {
	{ -0.382683,  0.923880, -1.000000,  1.000000 },
	{  0.382683, -0.923880, -1.000000,  1.000000 },
	{ -0.923880,  0.382683,  1.000000,  1.000000 },
	{ -0.382683, -0.923880,  1.000000,  1.000000 },
	{  0.382683,  0.923880,  1.000000,  1.000000 },
	{  0.923880, -0.382683,  1.000000,  1.000000 },
	{  0.923880,  0.382683, -1.000000,  1.000000 },
	{ -0.923880, -0.382683, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_octa_plus[] = {
	{  0.000000,  1.000000, -1.000000,  1.000000 },
	{ -0.000000, -1.000000, -1.000000,  1.000000 },
	{ -0.707107,  0.707107,  1.000000,  1.000000 },
	{ -0.707107, -0.707107,  1.000000,  1.000000 },
	{  0.707107,  0.707107,  1.000000,  1.000000 },
	{  0.707107, -0.707107,  1.000000,  1.000000 },
	{  1.000000,  0.000000, -1.000000,  1.000000 },
	{ -1.000000,  0.000000, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_octa_cox[] = {
	{ -0.707107,  0.707107,  1.000000,  1.000000 },
	{  0.707107,  0.707107, -1.000000,  1.000000 },
	{  0.707107, -0.707107,  1.000000,  1.000000 },
	{ -0.707107, -0.707107, -1.000000,  1.000000 },
	{  0.707107,  0.707107,  1.000000,  1.000000 },
	{ -0.707107,  0.707107, -1.000000,  1.000000 },
	{ -0.707107, -0.707107,  1.000000,  1.000000 },
	{  0.707107, -0.707107, -1.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_twin_engine[] = {
	{ -1.000000,  0.000000,  0.000000,  1.000000 },
	{  1.000000,  0.000000,  0.000000,  1.000000 },
};

const MultirotorMixer::Rotor _config_tri_y[] = {
	{ -0.866025,  0.500000,  0.000000,  1.000000 },
	{  0.866025,  0.500000,  0.000000,  1.000000 },
	{ -0.000000, -1.000000,  0.000000,  1.000000 },
};

const MultirotorMixer::Rotor *_config_index[] = {
	&_config_quad_x[0],
	&_config_quad_h[0],
	&_config_quad_plus[0],
	&_config_quad_v[0],
	&_config_quad_wide[0],
	&_config_quad_danaus[0],
	&_config_quad_deadcat[0],
	&_config_hex_x[0],
	&_config_hex_plus[0],
	&_config_hex_cox[0],
	&_config_octa_x[0],
	&_config_octa_plus[0],
	&_config_octa_cox[0],
	&_config_twin_engine[0],
	&_config_tri_y[0],
};

const unsigned _config_rotor_count[] = {
	4, /* quad_x */
	4, /* quad_h */
	4, /* quad_plus */
	4, /* quad_v */
	4, /* quad_wide */
	4, /* quad_danaus */
	4, /* quad_deadcat */
	6, /* hex_x */
	6, /* hex_plus */
	6, /* hex_cox */
	8, /* octa_x */
	8, /* octa_plus */
	8, /* octa_cox */
	2, /* twin_engine */
	3, /* tri_y */
};

} // anonymous namespace

#endif /* _MIXER_MULTI_TABLES */

