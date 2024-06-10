#ifndef MCP3XXX_H
#define MCP3XXX_H

#include <type_traits>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

enum mcp_adc_channels_t : uint8_t
{
	MCP_ADC_CHANNELS_2 = 2,
	MCP_ADC_CHANNELS_4 = 4,
	MCP_ADC_CHANNELS_8 = 8,
};

enum mcp_adc_bits_t : uint8_t
{
	MCP_ADC_BITS_10 = 10,
	MCP_ADC_BITS_12 = 12,
	MCP_ADC_BITS_13 = 13,
};

enum mcp_adc_signed_t : bool
{
	MCP_ADC_DATA_UNSIGNED = 0,
	MCP_ADC_DATA_SIGNED = 1
};

enum mcp_adc_read_mode_t : bool
{
	MCP_ADC_READ_DFRNTL = 0,
	MCP_ADC_READ_SINGLE = 1
};

//

template <mcp_adc_channels_t C, mcp_adc_bits_t B, mcp_adc_signed_t S = MCP_ADC_DATA_UNSIGNED>
class MCP3xxx
{
	static constexpr const char *const TAG = "MCP3xxx";

	using uout_t = uint16_t;
	using sout_t = int16_t;

public:
	using out_t = std::conditional_t<S, sout_t, uout_t>;
	static constexpr uint8_t bits = B;
	static constexpr uint8_t channels = C;
	static constexpr bool is_signed = S;

	static constexpr out_t ref = 1u << (bits - is_signed);
	static constexpr out_t max = ref - 1;
	static constexpr out_t min = ref - (1u << bits);

private:
	static constexpr uout_t resp_mask = (1u << bits) - 1;
	static constexpr uout_t sign_mask = is_signed ? (1u << (bits - 1)) : 0; // only used for signed anyway, but w/e

	spi_host_device_t spi_host;
	gpio_num_t cs_gpio;
	int clk_hz;
	spi_device_handle_t spi_hdl;

public:
	MCP3xxx(spi_host_device_t sh, gpio_num_t csg, int chz = 1'000'000) : spi_host(sh), cs_gpio(csg), clk_hz(chz)
	{
		ESP_LOGI(TAG, "Constructed with host: %d, pin: %d", spi_host, cs_gpio);
	}
	~MCP3xxx() = default;

	esp_err_t init()
	{
		assert(!spi_hdl);

		const spi_device_interface_config_t dev_cfg = {
			.command_bits = 2,
			.address_bits = (C == MCP_ADC_CHANNELS_2) ? 1 : 3,
			.dummy_bits = 2,
			.mode = 0,
			.duty_cycle_pos = 0,
			.cs_ena_pretrans = 0,
			.cs_ena_posttrans = 0,
			.clock_speed_hz = clk_hz,
			.input_delay_ns = 0,
			.spics_io_num = cs_gpio,
			.flags = SPI_DEVICE_HALFDUPLEX, // SPI_DEVICE_NO_DUMMY - dummy is required and used as the conversion time delay
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

		ESP_LOGE(TAG, "Error in spi_device_polling_end!");
		return ret;
	}

	inline out_t parse_trx(const spi_transaction_t &trx) const
	{
		uout_t out = SPI_SWAP_DATA_RX(*reinterpret_cast<const uint32_t *>(trx.rx_data), B);

		// ESP_LOGW(TAG, "0x%.8lx", *reinterpret_cast<const uint32_t *>(trx.rx_data));
		// ESP_LOGW(TAG, "0x%.4x", out);

		if constexpr (S)		   // if signed
			if (out & sign_mask)   // if negative
				out |= ~resp_mask; // flip high sign bits

		return out;
	}

	static spi_transaction_t make_trx(uint8_t chnl, mcp_adc_read_mode_t rdmd = MCP_ADC_READ_SINGLE)
	{
		// Request/Response format (tx/rx).
		//
		//   1 SG D2 D1 D0  0  0
		// |--|--|--|--|--|--|--|
		//
		// |--|--|--|--|--|--|--|--|--|--|--|--|
		//  12 11 10  9  8  7  6  5  4  3  2  1
		//
		// 3002      https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/21294E.pdf
		// 3004/3008 https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/MCP3004-MCP3008-Data-Sheet-DS20001295.pdf
		// 3202      https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/21034F.pdf
		// 3204/3208 https://ww1.microchip.com/downloads/en/devicedoc/21298e.pdf
		// 3302/3304 https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/21697F.pdf

		spi_transaction_t trx;

		trx.cmd = 0b10 | rdmd; // start bit and read mode
		trx.addr = chnl;	   // &= (C == MCP_CHANNELS_2) ? 0b1 : 0b111; but wil be truncated anyway
		trx.flags = SPI_TRANS_USE_RXDATA;

		trx.tx_buffer = nullptr;
		trx.length = 0;
		trx.rxlength = bits;

		return trx;
	}
};

// http://ww1.microchip.com/downloads/en/DeviceDoc/21294E.pdf
using MCP3002 = MCP3xxx<MCP_ADC_CHANNELS_2, MCP_ADC_BITS_10>;

// http://ww1.microchip.com/downloads/en/DeviceDoc/21295C.pdf
using MCP3004 = MCP3xxx<MCP_ADC_CHANNELS_4, MCP_ADC_BITS_10>;
using MCP3008 = MCP3xxx<MCP_ADC_CHANNELS_8, MCP_ADC_BITS_10>;

// http://ww1.microchip.com/downloads/en/DeviceDoc/21034D.pdf
using MCP3202 = MCP3xxx<MCP_ADC_CHANNELS_2, MCP_ADC_BITS_12>;

// http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf
using MCP3204 = MCP3xxx<MCP_ADC_CHANNELS_4, MCP_ADC_BITS_12>;
using MCP3208 = MCP3xxx<MCP_ADC_CHANNELS_8, MCP_ADC_BITS_12>;

// http://ww1.microchip.com/downloads/en/DeviceDoc/21697e.pdf
using MCP3302 = MCP3xxx<MCP_ADC_CHANNELS_4, MCP_ADC_BITS_13, MCP_ADC_DATA_SIGNED>;
using MCP3304 = MCP3xxx<MCP_ADC_CHANNELS_8, MCP_ADC_BITS_13, MCP_ADC_DATA_SIGNED>;

#endif
