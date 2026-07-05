#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>

#include "SoapyHPSDR.hpp"

/***********************************************************************
 * Find available devices
 **********************************************************************/

SoapySDR::KwargsList findMyHPSDR(const SoapySDR::Kwargs &args)
{
	SoapySDR::Kwargs options;

	static std::vector<SoapySDR::Kwargs> results;

	options["driver"] = "hpsdr";
	results.push_back(options);

	return results;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeMyHPSDR(const SoapySDR::Kwargs &args)
{
	return new SoapyHPSDR(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerHPSDR("hpsdr", &findMyHPSDR, &makeMyHPSDR, SOAPY_SDR_ABI_VERSION);
