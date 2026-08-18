/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <psa/crypto.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/pm/device.h>


#include "soil_sensor_auth_key.h"
#include "soil_sensor.h"

#define SLEEP_TIME_SEC 60
#define SOIL_SENSOR_COMPANY_ID 0xFFFFU
#define ADV_ELEM_OVERHEAD 2U
#define ADV_FLAGS_DATA_SIZE 1U
#define LEGACY_ADV_MAX_DATA_LEN 31U

struct soil_sensor_adv_payload
{
	uint16_t company_id;
	uint16_t device_id;
	uint32_t boot_nonce;
	uint16_t sample_counter;
	int32_t raw[SOIL_SENSOR_COUNT];
	uint32_t auth_tag;
} __packed;

static struct soil_sensor_adv_payload adv_payload = {
	.company_id = SOIL_SENSOR_COMPANY_ID,
};
static psa_key_id_t soil_sensor_psa_key_id = PSA_KEY_ID_NULL;

BUILD_ASSERT((ADV_ELEM_OVERHEAD + ADV_FLAGS_DATA_SIZE +
			  ADV_ELEM_OVERHEAD + sizeof(struct soil_sensor_adv_payload)) <=
				 LEGACY_ADV_MAX_DATA_LEN,
			 "Advertising payload exceeds 31-byte legacy limit");

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_NCONN;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &adv_payload, sizeof(adv_payload)),
};

static uint16_t soil_sensor_read_device_id(void)
{
	uint8_t unique_id[16];
	ssize_t unique_id_len = hwinfo_get_device_id(unique_id, sizeof(unique_id));
	uint16_t folded_id = 0U;

	if (unique_id_len <= 0)
	{
		return 1U;
	}

	for (ssize_t i = 0; i < unique_id_len; i += 2)
	{
		uint16_t part = unique_id[i];

		if ((i + 1) < unique_id_len)
		{
			part |= (uint16_t)unique_id[i + 1] << 8;
		}

		folded_id ^= part;
	}

	return (folded_id == 0U) ? 1U : folded_id;
}

static uint32_t soil_sensor_new_boot_nonce(void)
{
	uint32_t nonce = sys_rand32_get();

	return (nonce == 0U) ? 1U : nonce;
}

static int soil_sensor_crypto_init(void)
{
	psa_status_t status;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;

	if (soil_sensor_psa_key_id != PSA_KEY_ID_NULL)
	{
		return 0;
	}

	status = psa_crypto_init();
	if (status != PSA_SUCCESS)
	{
		return -EIO;
	}

	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
	psa_set_key_algorithm(&attributes, PSA_ALG_HMAC(PSA_ALG_SHA_256));
	psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
	psa_set_key_bits(&attributes, SOIL_SENSOR_AUTH_KEY_SIZE * 8U);

	status = psa_import_key(&attributes,
							soil_sensor_auth_key,
							sizeof(soil_sensor_auth_key),
							&soil_sensor_psa_key_id);
	psa_reset_key_attributes(&attributes);

	if (status != PSA_SUCCESS)
	{
		return -EIO;
	}

	return 0;
}

static int soil_sensor_adv_sign(void)
{
	uint8_t auth_digest[32];
	const size_t signed_len = offsetof(struct soil_sensor_adv_payload, auth_tag);
	size_t auth_digest_len = 0U;
	psa_status_t status;

	if (soil_sensor_psa_key_id == PSA_KEY_ID_NULL)
	{
		return -EINVAL;
	}

	status = psa_mac_compute(soil_sensor_psa_key_id,
							 PSA_ALG_HMAC(PSA_ALG_SHA_256),
							 (const uint8_t *)&adv_payload,
							 signed_len,
							 auth_digest,
							 sizeof(auth_digest),
							 &auth_digest_len);
	if ((status != PSA_SUCCESS) || (auth_digest_len < sizeof(uint32_t)))
	{
		return -EIO;
	}

	adv_payload.auth_tag = sys_get_le32(auth_digest);

	return 0;
}

static int soil_sensor_adv_update(void)
{
	int ret;

	ret = soil_sensor_crypto_init();
	if (ret < 0)
	{
		return ret;
	}

	adv_payload.device_id = soil_sensor_read_device_id();
	adv_payload.boot_nonce = soil_sensor_new_boot_nonce();
	adv_payload.sample_counter = 0U;

	for (size_t i = 0; i < SOIL_SENSOR_COUNT; ++i)
	{
		adv_payload.raw[i] = soil_sensor_get_raw(i);
	}

	return soil_sensor_adv_sign();
}

int main(void)
{
	int ret;

	printk("main: start\n");

	ret = soil_sensor_init();
	printk("soil_sensor_init: %d\n", ret);
	if (ret < 0)
	{
		return ret;
	}

	ret = bt_enable(NULL);
	printk("bt_enable: %d\n", ret);
	if (ret < 0)
	{
		return ret;
	}

	printk("sampling loop\n");
	while (1)
	{
		soil_sensor_sample();
		if (soil_sensor_has_new_data())
		{
			ret = soil_sensor_adv_update();
			printk("adv_update: %d\n", ret);
			break;
		}
	}
	

	ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	printk("bt_le_adv_start: %d\n", ret);

	k_sleep(K_MSEC(500));

	bt_le_adv_stop();

	printk("done\n");

	// sys_poweroff();
	return 0;
}
