
#include <esp_log.h>
#include <esp_check.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

enum mcp_channels_t : uint8_t
{
	MCP_CHANNELS_2 = 2,
	MCP_CHANNELS_4 = 4,
	MCP_CHANNELS_8 = 8,
};

enum mcp_bits_t : uint8_t
{
	MCP_BITS_10 = 10,
	MCP_BITS_12 = 12,
	MCP_BITS_13 = 13,
};

enum mcp_read_mode_t : bool
{
	MCP_READ_DFRNTL = 0,
	MCP_READ_SINGLE = 1
};

enum mcp_signed_t : bool
{
	MCP_DATA_UNSIGNED = 0,
	MCP_DATA_SIGNED = 1
};

#define TAG "MCP3xxx"

template <mcp_channels_t C, mcp_bits_t B, mcp_signed_t S = MCP_DATA_UNSIGNED>
class MCP3xxx
{
	spi_device_handle_t spi_hdl;
	int32_t ref_mvolt;
	float scale;

	static constexpr uint8_t requ_len = 2 + ((C == MCP_CHANNELS_2) ? 1 : 3) + 2;
	static constexpr uint8_t totl_len = requ_len + B;
	static constexpr uint8_t bytes_len = (totl_len - 1) / 8 + 1;
	static constexpr int32_t resp_mask = (1u << B) - 1;

public:
	static constexpr int32_t raw_max = S ? (1u << (B - 1)) : (1u << B);
	static constexpr int32_t raw_min = S ? (1u << (B - 1)) - 1 : 0;

	MCP3xxx(spi_host_device_t spihost, gpio_num_t csgpio, int clkhz = 1'000'000, int rmv = 5000, float sc = 1) : ref_mvolt(rmv), scale(sc)
	{
		esp_err_t ret = ESP_OK;

		const spi_device_interface_config_t dev_cfg = {
			.command_bits = 0,
			.address_bits = 0,
			.mode = 0,
			.clock_speed_hz = clkhz,
			.input_delay_ns = 0,
			.spics_io_num = csgpio,
			.flags = SPI_DEVICE_NO_DUMMY,
			.queue_size = 1,
			.pre_cb = NULL,
			.post_cb = NULL,

		};

		ESP_GOTO_ON_ERROR(
			spi_bus_add_device(spihost, &dev_cfg, &spi_hdl),
			err, TAG, "Failed to add device to SPI bus!");
		return;

	err:
		ESP_LOGE(TAG, "Failed to construct MCP! Error: %s", esp_err_to_name(ret));
	}
	~MCP3xxx()
	{
		esp_err_t ret = ESP_OK;

		ESP_GOTO_ON_ERROR(
			spi_bus_remove_device(spi_hdl),
			err, TAG, "Failed to remove device from SPI bus!");
		return;

	err:
		ESP_LOGE(TAG, "Failed to destruct MCP! Error: %s", esp_err_to_name(ret));
	}
	//

	esp_err_t acquire_spi(TickType_t timeout)
	{
		ESP_RETURN_ON_ERROR(
			spi_device_acquire_bus(spi_hdl, timeout),
			TAG, "Failed to acquire SPI bus!");

		return ESP_OK;
	}

	esp_err_t release_spi()
	{
		ESP_RETURN_ON_ERROR(
			spi_device_release_bus(spi_hdl),
			TAG, "Failed to acquire SPI bus!");

		return ESP_OK;
	}

	int32_t get_signed_raw(uint8_t chnl, mcp_read_mode_t rdmd = MCP_READ_SINGLE, size_t scnt = 1)
	{

		spi_transaction_t spi_trx = make_transaction(chnl, rdmd);

		int32_t sum = 0;
		int32_t raw;

		for (size_t i = 0; i < scnt; ++i)
		{
			raw = get_response(&spi_trx);
			if (raw == -1)
				goto errgs;
			sum += raw;
		}

		return (sum + ((sum < 0) ? -scnt : scnt) / 2) / scnt;

	errgs:
		return 0;
	}

	float get_float_volt(uint8_t chnl, mcp_read_mode_t rdmd = MCP_READ_SINGLE, size_t scnt = 1)
	{
		spi_transaction_t spi_trx = make_transaction(chnl, rdmd);

		int32_t sum = 0;
		int32_t raw;

		for (size_t i = 0; i < scnt; ++i)
		{
			raw = get_response(&spi_trx);
			if (raw == -1)
				goto errgs;
			sum += raw;
		}

		return sum / 1000.0f / scnt * ref_mvolt / raw_max * scale;

	errgs:
		return 0;
	}

private:
	int32_t get_response(spi_transaction_t *spi_trx)
	{
		esp_err_t ret = ESP_OK;

		int32_t recv = 0;
		uint8_t *bytes = reinterpret_cast<uint8_t *>(&recv);

		ESP_GOTO_ON_ERROR(
			spi_device_transmit(spi_hdl, spi_trx),
			errgr, TAG, "Error in get_response()");

		switch (bytes_len)
		{
		case 4:
			bytes[3] = spi_trx->rx_data[0];
			bytes[2] = spi_trx->rx_data[1];
			bytes[1] = spi_trx->rx_data[2];
			bytes[0] = spi_trx->rx_data[3];
			ESP_LOGV(TAG, "SPI 4 bytes!");
			break;
		case 3:
			bytes[2] = spi_trx->rx_data[0];
			bytes[1] = spi_trx->rx_data[1];
			bytes[0] = spi_trx->rx_data[2];
			ESP_LOGV(TAG, "SPI 3 bytes!");
			break;
		case 2:
			bytes[1] = spi_trx->rx_data[0];
			bytes[0] = spi_trx->rx_data[1];
			ESP_LOGV(TAG, "SPI 2 bytes!");
			break;
		default:
			bytes[0] = spi_trx->rx_data[0];
			ESP_LOGV(TAG, "SPI 1 bytes!");
			break;
		};

		recv &= resp_mask;

		if constexpr (S == MCP_DATA_SIGNED) // if signed
			if (recv >> B - 1)				// if negative
				recv |= ~resp_mask;

		return recv;

	errgr:
		ESP_LOGE(TAG, "Error: %s", esp_err_to_name(ret));
		return 0;
	}

	spi_transaction_t make_transaction(uint8_t chnl, mcp_read_mode_t rdmd)
	{
		// Request/Response format (tx/rx), eight bits aligned.
		//
		//  0  0  0  0  0  1  SG D2     D1 D0 X  X  X  X  X  X      X  X  X  X  X  X  X  X
		// |--|--|--|--|--|--|--|--|   |--|--|--|--|--|--|--|--|   |--|--|--|--|--|--|--|--|
		//  X  X  X  X  X  X  X  X      X  X  X  0  BY BX B9 B8     B7 B6 B5 B4 B3 B2 B1 B0
		//
		// Where Request:
		//   * 0: dummy bits, must be zero.
		//   * 1: start bit.
		//   * SG/~DIFF:
		//     - 0: differential conversion.
		//     - 1: single conversion.
		//   * D [2 1 0]: three bits for selecting 8 channels, for 4 the D2 is X
		//   * X: dummy bits, whatever value.
		//
		// Where Response:
		//   * X: dummy bits; whatever value.
		//   * 0: start bit.
		//   * B [0 1 2 3 4 5 6 7 8 9 X Y Z]: big-endian.
		//     - BY: most significant bit.
		//     - B0: least significant bit.

		uint32_t send = 5;
		uint8_t *bytes = reinterpret_cast<uint8_t *>(&send);

		chnl &= (C == MCP_CHANNELS_2) ? 0b1 : 0b111;

		// start bit
		send |= 1 << (totl_len - 1);
		// single/diff
		send |= rdmd << (totl_len - 2);
		// channel
		send |= chnl << (totl_len - requ_len + 2);

		spi_transaction_t spi_trx = {
			.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,
		};

		spi_trx.length = bytes_len * 8;

		switch (bytes_len)
		{
		case 4:
			spi_trx.tx_data[0] = bytes[3];
			spi_trx.tx_data[1] = bytes[2];
			spi_trx.tx_data[2] = bytes[1];
			spi_trx.tx_data[3] = bytes[0];
			ESP_LOGV(TAG, "SPI 4 bytes!");
			break;
		case 3:
			spi_trx.tx_data[0] = bytes[2];
			spi_trx.tx_data[1] = bytes[1];
			spi_trx.tx_data[2] = bytes[0];
			ESP_LOGV(TAG, "SPI 3 bytes!");
			break;
		case 2:
			spi_trx.tx_data[0] = bytes[1];
			spi_trx.tx_data[1] = bytes[0];
			ESP_LOGV(TAG, "SPI 2 bytes!");
			break;
		default:
			spi_trx.tx_data[0] = bytes[0];
			ESP_LOGV(TAG, "SPI 1 bytes!");
			break;
		};

		return spi_trx;
	}
};

#undef TAG

/// \brief A typedef for the MCP3002.
/// Max clock frequency for 2.7V: 1200000 Hz
/// Max clock frequency for 5.0V: 3200000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21294E.pdf
using MCP3002 = MCP3xxx<MCP_CHANNELS_2, MCP_BITS_10>;

/// \brief A typedef for the MCP3004.
/// Max clock frequency for 2.7V: 1350000 Hz
/// Max clock frequency for 5.0V: 3600000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21295C.pdf
using MCP3004 = MCP3xxx<MCP_CHANNELS_4, MCP_BITS_10>;

/// \brief A typedef for the MCP3008.
/// Max clock frequency for 2.7V: 1350000 Hz
/// Max clock frequency for 5.0V: 3600000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21295C.pdf
using MCP3008 = MCP3xxx<MCP_CHANNELS_8, MCP_BITS_10>;

/// \brief A typedef for the MCP3202.
/// Max clock frequency for 2.7V:  900000 Hz
/// Max clock frequency for 5.0V: 1800000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21034D.pdf
using MCP3202 = MCP3xxx<MCP_CHANNELS_2, MCP_BITS_12>;

/// \brief A typedef for the MCP3204.
/// Max clock frequency for 2.7V: 1000000 Hz
/// Max clock frequency for 5.0V: 2000000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf
using MCP3204 = MCP3xxx<MCP_CHANNELS_4, MCP_BITS_12>;

/// \brief A typedef for the MCP3208.
/// Max clock frequency for 2.7V: 1000000 Hz
/// Max clock frequency for 5.0V: 2000000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21298c.pdf
using MCP3208 = MCP3xxx<MCP_CHANNELS_8, MCP_BITS_12>;

/// \brief A typedef for the MCP3302.
/// Max clock frequency for 2.7V: 1050000 Hz
/// Max clock frequency for 5.0V: 2100000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21697e.pdf
using MCP3302 = MCP3xxx<MCP_CHANNELS_4, MCP_BITS_13, MCP_DATA_SIGNED>;

/// \brief A typedef for the MCP3304.
/// Max clock frequency for 2.7V: 1050000 Hz
/// Max clock frequency for 5.0V: 2100000 Hz
/// \sa http://ww1.microchip.com/downloads/en/DeviceDoc/21697e.pdf
using MCP3304 = MCP3xxx<MCP_CHANNELS_8, MCP_BITS_13, MCP_DATA_SIGNED>;