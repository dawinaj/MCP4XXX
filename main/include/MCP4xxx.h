#ifndef MCP4XXX_H
#define MCP4XXX_H

#include <algorithm>
#include <type_traits>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

enum mcp_dac_channels_t : uint8_t
{
	MCP_DAC_CHANNELS_1 = 1,
	MCP_DAC_CHANNELS_2 = 2,
};

enum mcp_dac_bits_t : uint8_t
{
	MCP_DAC_BITS_8 = 8,
	MCP_DAC_BITS_10 = 10,
	MCP_DAC_BITS_12 = 12,
};

template <mcp_dac_channels_t C, mcp_dac_bits_t B>
class MCP4xxx
{
	static constexpr const char *const TAG = "MCP4xxx";

public:
	using in_t = uint16_t;
	static constexpr uint8_t bits = B;
	static constexpr uint8_t channels = C;

	static constexpr in_t ref = (1u << bits);
	static constexpr in_t max = ref - 1;
	static constexpr in_t min = 0;

private:
	static constexpr in_t inp_mask = (1u << bits) - 1;

	spi_host_device_t spi_host;
	gpio_num_t cs_gpio;
	int clk_hz;
	spi_device_handle_t spi_hdl = nullptr;

public:
	MCP4xxx(spi_host_device_t sh, gpio_num_t csg, int chz = 10'000'000) : spi_host(sh), cs_gpio(csg), clk_hz(chz)
	{
		ESP_LOGI(TAG, "Constructed with host: %d, pin: %d", spi_host, cs_gpio);
	}
	~MCP4xxx() = default;

	esp_err_t init()
	{
		assert(!spi_hdl);

		const spi_device_interface_config_t dev_cfg = {
			.command_bits = 4,
			.address_bits = 0,
			.dummy_bits = 0,
			.mode = 0,
			.duty_cycle_pos = 0,
			.cs_ena_pretrans = 0,
			.cs_ena_posttrans = 0,
			.clock_speed_hz = clk_hz,
			.input_delay_ns = 0,
			.spics_io_num = cs_gpio,
			.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
			.queue_size = 1,
			.pre_cb = NULL,
			.post_cb = NULL,
		};

		ESP_RETURN_ON_ERROR(
			spi_bus_add_device(spi_host, &dev_cfg, &spi_hdl),
			TAG, "Error in spi_bus_add_device!");

		return ESP_OK;
	}

	esp_err_t deinit()
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_bus_remove_device(spi_hdl),
			TAG, "Error in spi_bus_remove_device!");
		spi_hdl = nullptr;

		return ESP_OK;
	}

	//

	esp_err_t acquire_spi(TickType_t timeout = portMAX_DELAY) const
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_device_acquire_bus(spi_hdl, timeout),
			TAG, "Error in spi_device_acquire_bus!");

		return ESP_OK;
	}

	esp_err_t release_spi() const
	{
		assert(spi_hdl);

		spi_device_release_bus(spi_hdl); // return void
		return ESP_OK;
	}

	//

	void shutdown_channel(bool chnl) const
	{
		spi_transaction_t trx = make_trx(chnl, true);
		send_trx(trx);
		recv_trx();
	}

	inline esp_err_t send_trx(spi_transaction_t &trx) const
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_device_polling_start(spi_hdl, &trx, portMAX_DELAY),
			TAG, "Error in spi_device_polling_start!");

		return ESP_OK;
	}

	inline esp_err_t recv_trx(TickType_t timeout = portMAX_DELAY) const
	{
		assert(spi_hdl);

		esp_err_t ret = spi_device_polling_end(spi_hdl, timeout);

		if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) [[likely]]
			return ret;

		ESP_LOGE(TAG, "Error in recv_trx!");
		return ret;
	}

	inline void write_trx(spi_transaction_t &trx, in_t in) const
	{
		*reinterpret_cast<uint32_t *>(trx.tx_data) = SPI_SWAP_DATA_TX(in, B);
	}

	static spi_transaction_t make_trx(bool chnl, bool shdn = false, bool gain_x2 = false, bool ref_buf = false)
	{
		// Request format (tx)
		//  _     __ __
		//  AB BF GA SD
		// |--|--|--|--|
		//
		//  12 11 10  9  8  7  6  5  4  3  2  1
		// |--|--|--|--|--|--|--|--|--|--|--|--|
		//
		// 4901/4911/4921 https://ww1.microchip.com/downloads/en/devicedoc/22248a.pdf page 24
		// 4902/4912/4922 https://ww1.microchip.com/downloads/en/devicedoc/22250a.pdf page 24
		//

		spi_transaction_t trx;

		trx.cmd = ((C == MCP_DAC_CHANNELS_2) ? chnl << 3 : 0) | ref_buf << 2 | !gain_x2 << 1 | !shdn;
		trx.flags = SPI_TRANS_USE_TXDATA;

		trx.rx_buffer = nullptr;
		trx.length = bits;
		trx.rxlength = 0;

		return trx;
	}
};

// https://ww1.microchip.com/downloads/en/DeviceDoc/22244B.pdf

using MCP4801 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_8>;
using MCP4811 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_10>;
using MCP4821 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_12>;

// https://ww1.microchip.com/downloads/en/DeviceDoc/20002249B.pdf

using MCP4802 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_8>;
using MCP4812 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_10>;
using MCP4822 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_12>;

//

// https://ww1.microchip.com/downloads/en/devicedoc/22248a.pdf

using MCP4901 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_8>;
using MCP4911 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_10>;
using MCP4921 = MCP4xxx<MCP_DAC_CHANNELS_1, MCP_DAC_BITS_12>;

// https://ww1.microchip.com/downloads/en/devicedoc/22250a.pdf

using MCP4902 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_8>;
using MCP4912 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_10>;
using MCP4922 = MCP4xxx<MCP_DAC_CHANNELS_2, MCP_DAC_BITS_12>;

#endif
