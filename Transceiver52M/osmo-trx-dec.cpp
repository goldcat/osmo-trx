/*
 * Copyright (C) 2016 Alexander Chemeris <Alexander.Chemeris@fairwaves.co>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <limits.h>
#include <fstream>

#include "Logger.h"
#include "sigProcLib.h"
#include "signalVector.h"
#include "Transceiver.h"
#include "Configuration.h"

#define DEFAULT_RX_SPS		1

ConfigurationTable gConfig;

/** Codes for burst types of received bursts*/
typedef enum {
  OFF,               ///< timeslot is off
  TSC,	       ///< timeslot should contain a normal burst
  RACH,	       ///< timeslot should contain an access burst
  EDGE,	       ///< timeslot should contain an EDGE burst
  IDLE	       ///< timeslot is an idle (or dummy) burst
} CorrType;

struct trx_config {
  std::string log_level;
  unsigned rx_sps;
  unsigned rtsc;
  unsigned max_expected_delay_nb;
  unsigned max_expected_delay_ab;
  double rx_full_scale;
  bool edge;
  CorrType type;
  std::string filename;
};

int detectBurst(const trx_config &config, signalVector &burst, complex &amp, float &toa, CorrType type)
{
  float threshold = 5.0, rc = 0;

  switch (type) {
  case EDGE:
    rc = detectEdgeBurst(burst, config.rtsc, threshold, config.rx_sps,
                         amp, toa, config.max_expected_delay_nb);
    if (rc > 0)
      break;
    else
      type = TSC;
  case TSC:
    rc = analyzeTrafficBurst(burst, config.rtsc, threshold, config.rx_sps,
                             amp, toa, config.max_expected_delay_nb);
    break;
  case RACH:
    threshold = 6.0;
    rc = detectRACHBurst(burst, threshold, config.rx_sps, amp, toa,
                         config.max_expected_delay_ab);
    break;
  case IDLE:
    rc = SIGERR_NONE;
    break;
  default:
    LOG(ERR) << "Invalid correlation type";
  }

  if (rc > 0)
    return type;

  return rc;
}


SoftVector *demodulateBurst(const trx_config &config, signalVector &burst, int SPSRx, double &RSSI,
                            double &timingOffset, CorrType type)
{
  int rc;
  complex amp;
  float toa, avg = 0.0;

  /* Calculate average power of the burst */
  energyDetect(burst, 20 * config.rx_sps, 0.0, &avg);
  RSSI = 20.0 * log10(config.rx_full_scale / avg);

  /* Detect normal or RACH bursts */
  rc = detectBurst(config, burst, amp, toa, type);

  if (rc > 0) {
    type = (CorrType) rc;
  } else if (rc <= 0) {
    if (rc == -SIGERR_CLIP) {
      std::cout << "Clipping detected on received RACH or Normal Burst";
    } else if (rc != SIGERR_NONE) {
      LOG(WARNING) << "Unhandled RACH or Normal Burst detection error";
    }
    return NULL;
  }

  timingOffset = toa / SPSRx;

  if (type == EDGE)
    return demodEdgeBurst(burst, config.rx_sps, amp, toa);
  else
    return demodulateBurst(burst, config.rx_sps, amp, toa);
}


// Setup configuration values
static void print_config(struct trx_config *config)
{
  std::ostringstream ost("");
  ost << "Config Settings" << std::endl;
  ost << "   Log Level............... " << config->log_level << std::endl;
  ost << "   Rx Samples-per-Symbol... " << config->rx_sps << std::endl;
  ost << "   EDGE support............ " << (config->edge ? "Enabled" : "Disabled") << std::endl;
  ost << "   Burts TSC............... " << config->rtsc << std::endl;
  std::cout << ost << std::endl;
}

static void print_help()
{
  fprintf(stdout, "Options:\n"
          "  -h          This text\n"
          "  -l LEVEL    Logging level (%s)\n"
          "  -e          Enable EDGE receiver\n"
          "  -s SPS      Samples-per-symbol (1 or 4)\n"
          "  -t TSC      Burst training sequence (1 to 7)\n"
          "  -f FILE     File to read\n",
          "EMERG, ALERT, CRT, ERR, WARNING, NOTICE, INFO, DEBUG");
}

static void handle_options(int argc, char **argv, struct trx_config *config)
{
  int option;

  config->log_level = "NOTICE";
  config->rx_sps = DEFAULT_RX_SPS;
  config->rtsc = 0;
  config->max_expected_delay_nb = 30;
  config->max_expected_delay_ab = 30;
  config->rx_full_scale = SHRT_MAX;
  config->edge = false;
  config->type = TSC;

  while ((option = getopt(argc, argv, "ls:et:f:h")) != -1) {
    switch (option) {
    case 'l':
      config->log_level = optarg;
      break;
    case 's':
      config->rx_sps = atoi(optarg);
      break;
    case 'e':
      config->edge = true;
      break;
    case 't':
      config->rtsc = atoi(optarg);
      break;
    case 'f':
      config->filename = optarg;
      break;
    case 'h':
    default:
      print_help();
      exit(0);
    }
  }

  if ((config->rx_sps != 1) && (config->rx_sps != 4)) {
    printf("Unsupported samples-per-symbol %i\n\n", config->rx_sps);
    print_help();
    exit(0);
  }

  if (config->edge && (config->rx_sps != 4)) {
    printf("EDGE only supported at 4 samples per symbol\n\n");
    print_help();
    exit(0);
  }

  if (config->rtsc > 7) {
    printf("Invalid training sequence %i\n\n", config->rtsc);
    print_help();
    exit(0);
  }
}

int main(int argc, char *argv[])
{
  struct trx_config config;

  // Process command line options and print config to screen
  handle_options(argc, argv, &config);
  print_config(&config);

  gLogInit("transceiver", config.log_level.c_str(), LOG_LOCAL7);

  signalVector burst(156);
  double RSSI;
  double timingOffset;

  std::ifstream file (config.filename.c_str(), std::ifstream::binary);
  file.read((char*)burst.begin(), burst.size() * 2 * sizeof(float));
  file.close();

  SoftVector *softBits = demodulateBurst(config, burst, config.rx_sps, RSSI, timingOffset, config.type);
  delete softBits;

  return 0;
}
