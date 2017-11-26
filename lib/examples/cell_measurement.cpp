/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <assert.h>
#include <signal.h>

#define ENABLE_AGC_DEFAULT

extern "C" {
#include "srslte/srslte.h"
#include "srslte/phy/rf/rf.h"
#include "srslte/phy/rf/rf_utils.h"
}

#include "srslte/common/common.h"
#include "srslte/asn1/liblte_rrc.h"

cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS, 
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};


static uint32 cell_measurement_decode_sib(uint8_t * data, uint32_t n);
static void cell_measurement_decode_sib1(const LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT * sib1);
static void cell_measurement_decode_sib2(const LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT * sib2);



/**********************************************************************
 *  Program arguments processing
 ***********************************************************************/
typedef struct {
  int nof_subframes;
  bool disable_plots;
  int force_N_id_2;
  char *rf_args;
  char * ref_sym_fname;
  float rf_freq; 
  float rf_gain;
}prog_args_t;

void args_default(prog_args_t *args) {
  args->nof_subframes = -1; 
  args->force_N_id_2 = -1; // Pick the best
  args->rf_args = "";
  args->rf_freq = -1.0;
	args->ref_sym_fname = "";
#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1; 
#else
  args->rf_gain = 50; 
#endif
}

void usage(prog_args_t *args, char *prog) {
  printf("Usage: %s [aglnv] -f rx_frequency (in Hz)\n", prog);
  printf("\t-a RF args [Default %s]\n", args->rf_args);
  printf("\t-g RF RX gain [Default %.2f dB]\n", args->rf_gain);
  printf("\t-l Force N_id_2 [Default best]\n");
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-v [set srslte_verbose to debug, default none]\n");
}

int  parse_args(prog_args_t *args, int argc, char **argv) {
  int opt;
  args_default(args);
  while ((opt = getopt(argc, argv, "aglnvrf")) != -1) {
    switch (opt) {
    case 'a':
      args->rf_args = argv[optind];
      break;
    case 'g':
      args->rf_gain = atof(argv[optind]);
      break;
    case 'f':
      args->rf_freq = atof(argv[optind]);
      break;
    case 'n':
      args->nof_subframes = atoi(argv[optind]);
			break;
    case 'r':
      args->ref_sym_fname = argv[optind];
      break;
    case 'l':
      args->force_N_id_2 = atoi(argv[optind]);
      break;
    case 'v':
      srslte_verbose++;
      break;
    default:
      usage(args, argv[0]);
      return -1;
    }
  }
  if (args->rf_freq < 0) {
    usage(args, argv[0]);
    return -1;
  }
  return 0;
}
/**********************************************************************/

/* TODO: Do something with the output data */
uint8_t *data[SRSLTE_MAX_CODEWORDS];

bool go_exit = false; 
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

int srslte_rf_recv_wrapper(void *h, cf_t *data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *q) {  
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  
  return srslte_rf_recv((srslte_rf_t *)h, data[0], nsamples, 1);
}

enum receiver_state { DECODE_MIB, DECODE_SIB, MEASURE} state; 

#define MAX_SINFO 10
#define MAX_NEIGHBOUR_CELLS     128

int main(int argc, char **argv) {
  int ret; 
  cf_t *sf_buffer[SRSLTE_MAX_PORTS] = {NULL, NULL}; 
  prog_args_t prog_args; 
  srslte_cell_t cell;  
  int64_t sf_cnt;
  srslte_ue_sync_t ue_sync; 
  srslte_ue_mib_t ue_mib; 
  srslte_rf_t rf; 
  srslte_ue_dl_t ue_dl; 
  srslte_ofdm_t fft; 
  srslte_chest_dl_t chest; 
  uint32_t nframes=0;
  uint32_t nof_trials = 0; 
  uint32_t sfn = 0; // system frame number
  int n; 
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int sfn_offset; 
  float rssi_utra=0,rssi=0, rsrp=0, rsrq=0, snr=0;
  cf_t *ce[SRSLTE_MAX_PORTS];
  float cfo = 0;
  bool acks[SRSLTE_MAX_CODEWORDS] = {false, false};
  FILE * crs = NULL;
	
  if (parse_args(&prog_args, argc, argv)) {
    exit(-1);
  }

  if(prog_args.ref_sym_fname)
  {
    crs = fopen(prog_args.ref_sym_fname, "w");
    if (crs)
      printf("Writing CRS to %s\n", prog_args.ref_sym_fname);
  }

  printf("Opening RF device...\n");
  if (srslte_rf_open(&rf, prog_args.rf_args)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }
  if (prog_args.rf_gain > 0) {
    srslte_rf_set_rx_gain(&rf, prog_args.rf_gain);      
  } else {
    printf("Starting AGC thread...\n");
    if (srslte_rf_start_gain_thread(&rf, false)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    srslte_rf_set_rx_gain(&rf, 50);
  }
  
  sf_buffer[0] = (cf_t *)srslte_vec_malloc(3*sizeof(cf_t)*SRSLTE_SF_LEN_PRB(100));
  for (int i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
    data[i] = (uint8_t *)srslte_vec_malloc(sizeof(uint8_t) * 1500*8);
  }

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

  /* set receiver frequency */
  srslte_rf_set_rx_freq(&rf, (double) prog_args.rf_freq);
  srslte_rf_rx_wait_lo_locked(&rf);
  printf("Tunning receiver to %.3f MHz\n", (double ) prog_args.rf_freq/1000000);
  
  cell_detect_config.init_agc = (prog_args.rf_gain<0);
  
  uint32_t ntrial=0; 
  do {
    ret = rf_search_and_decode_mib(&rf, 1, &cell_detect_config, prog_args.force_N_id_2, &cell, &cfo);
    if (ret < 0) {
      fprintf(stderr, "Error searching for cell\n");
      exit(-1); 
    } else if (ret == 0 && !go_exit) {
      printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
    }      
  } while (ret == 0 && !go_exit); 
  
  if (go_exit) {
    exit(0);
  }
  
  /* set sampling frequency */
    int srate = srslte_sampling_freq_hz(cell.nof_prb);    
    if (srate != -1) {  
      if (srate < 10e6) {          
        srslte_rf_set_master_clock_rate(&rf, 4*srate);        
      } else {
        srslte_rf_set_master_clock_rate(&rf, srate);        
      }
      printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
      float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
      if (srate_rf != srate) {
        fprintf(stderr, "Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      fprintf(stderr, "Invalid number of PRB %d\n", cell.nof_prb);
      exit(-1);
    }

  INFO("Stopping RF and flushing buffer...\n",0);
  srslte_rf_stop_rx_stream(&rf);
  srslte_rf_flush_buffer(&rf);
  
  if (srslte_ue_sync_init_multi(&ue_sync, cell.nof_prb, cell.id==1000, srslte_rf_recv_wrapper, 1, (void*) &rf)) {
    fprintf(stderr, "Error initiating ue_sync\n");
    return -1; 
  }
  if (srslte_ue_sync_set_cell(&ue_sync, cell)) {
    fprintf(stderr, "Error initiating ue_sync\n");
    return -1;
  }
  if (srslte_ue_dl_init(&ue_dl, cell.nof_prb, 1)) {
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    return -1;
  }
  if (srslte_ue_dl_set_cell(&ue_dl, cell)) {
    fprintf(stderr, "Error initiating UE downlink processing module\n");
    return -1;
  }
  if (srslte_ue_mib_init(&ue_mib, cell.nof_prb)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    return -1;
  }
  if (srslte_ue_mib_set_cell(&ue_mib, cell)) {
    fprintf(stderr, "Error initaiting UE MIB decoder\n");
    return -1;
  }

  /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
  srslte_ue_dl_set_rnti(&ue_dl, SRSLTE_SIRNTI); 

  /* Initialize subframe counter */
  sf_cnt = 0;
    
  if (srslte_ofdm_rx_init(&fft, cell.cp, cell.nof_prb)) {
    fprintf(stderr, "Error initiating FFT\n");
    return -1;
  }
  if (srslte_chest_dl_init(&chest, cell.nof_prb)) {
    fprintf(stderr, "Error initiating channel estimator\n");
    return -1;
  }
  if (srslte_chest_dl_set_cell(&chest, cell)) {
    fprintf(stderr, "Error initiating channel estimator\n");
    return -1;
  }

  int sf_re = SRSLTE_SF_LEN_RE(cell.nof_prb, cell.cp);

  cf_t *sf_symbols = (cf_t *)srslte_vec_malloc(sf_re * sizeof(cf_t));

  for (int i=0;i<SRSLTE_MAX_PORTS;i++) {
    ce[i] = (cf_t *)srslte_vec_malloc(sizeof(cf_t) * sf_re);
  }
  
  srslte_rf_start_rx_stream(&rf);
  
  float rx_gain_offset = 0;

  // Set initial CFO for ue_sync
  srslte_ue_sync_set_cfo(&ue_sync, cfo); 

  /* Main loop */
  while ((sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1) && !go_exit) {
    
    ret = srslte_ue_sync_zerocopy_multi(&ue_sync, sf_buffer);
    if (ret < 0) {
      fprintf(stderr, "Error calling srslte_ue_sync_work()\n");
    }

        
    /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
    if (ret == 1) {
      switch (state) {
        case DECODE_MIB:
          if (srslte_ue_sync_get_sfidx(&ue_sync) == 0) {
            srslte_pbch_decode_reset(&ue_mib.pbch);
            n = srslte_ue_mib_decode(&ue_mib, sf_buffer[0], bch_payload, NULL, &sfn_offset);
            if (n < 0) {
              fprintf(stderr, "Error decoding UE MIB\n");
              return -1;
            } else if (n == SRSLTE_UE_MIB_FOUND) {   
              srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
              printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              sfn = (sfn + sfn_offset)%1024; 
              state = DECODE_SIB; 
            }
          }
          break;
        case DECODE_SIB:
          /* We are looking for SI Blocks, search only in appropiate places */
          if (1){ //(srslte_ue_sync_get_sfidx(&ue_sync) == 5 && (sfn%2)==0)) {

            memset(data[0], 0, 8 * 1500 * sizeof(uint8));
            memset(data[1], 0, 8 * 1500 * sizeof(uint8));
            acks[0] = false;
            acks[1] = false;
              
            n = srslte_ue_dl_decode(&ue_dl, sf_buffer, data, 0, sfn*10+srslte_ue_sync_get_sfidx(&ue_sync), acks);
            if (n < 0) {
              fprintf(stderr, "Error decoding UE DL\n");fflush(stdout);
              return -1;
            } else if (n == 0) {
              printf("CFO: %+6.4f kHz, SFO: %+6.4f kHz, PDCCH-Det: %.3f\r",
                      srslte_ue_sync_get_cfo(&ue_sync)/1000, srslte_ue_sync_get_sfo(&ue_sync)/1000, 
                      (float) ue_dl.nof_detected/nof_trials);
              nof_trials++; 
            } else {
              
              srslte_vec_fprint_byte(stdout, data[0], n/8);

              if (acks[0])
              {
                uint32 sib_type =  cell_measurement_decode_sib(data[0], n);
                static uint32 sibs_rxd = 0;
                const uint32 sibs_rqd = (1 << LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1) | (1 << LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2) | (1 << LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5) | (1 << LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6);

                if(sib_type)
                  sibs_rxd |= (sib_type);

                printf("SIBs RXd=%04x RQd=%04x\n", sibs_rxd, sibs_rqd);
                if((sibs_rxd & sibs_rqd) == sibs_rqd)
                {
                  state = MEASURE;
                }
              }
              else
              {
                printf("BAD CRC\n");
              }
            }
          }
        break;
        
      case MEASURE:
        if (srslte_ue_sync_get_sfidx(&ue_sync) == 5 || (sfn%128) == 127) {
          /* Run FFT for all subframe data */
          srslte_ofdm_rx_sf(&fft, sf_buffer[0], sf_symbols);
          
          srslte_chest_dl_estimate(&chest, sf_symbols, ce, srslte_ue_sync_get_sfidx(&ue_sync));
                  
          rssi = SRSLTE_VEC_EMA(srslte_vec_avg_power_cf(sf_buffer[0],SRSLTE_SF_LEN(srslte_symbol_sz(cell.nof_prb))),rssi,0.05);
          rssi_utra = SRSLTE_VEC_EMA(srslte_chest_dl_get_rssi(&chest),rssi_utra,0.05);
          rsrq = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrq(&chest),rsrq,0.05);
          rsrp = SRSLTE_VEC_EMA(srslte_chest_dl_get_rsrp(&chest),rsrp,0.05);      
          snr = SRSLTE_VEC_EMA(srslte_chest_dl_get_snr(&chest),snr,0.05);      
          
          nframes++;          

          if (crs && ((sfn%128) == 127))
          {
            int qq = 0;
            fprintf(crs, "[ID=%d SFN=%d SF=%d NPRB=%d Nports=%d CP=%d PHICHlen=%d PHICHres=%d]\n", 
                      ue_sync.cell.id, sfn, srslte_ue_sync_get_sfidx(&ue_sync), 
                      ue_sync.cell.nof_prb, ue_sync.cell.nof_ports, ue_sync.cell.cp, 
                      ue_sync.cell.phich_length, ue_sync.cell.phich_resources);
          
            for(qq = 0; qq< 120; ++qq)
            {
              fprintf(crs, "%f, %f\n", crealf(chest.pilot_recv_signal[qq]), cimagf(chest.pilot_recv_signal[qq]));
            }
            fprintf(crs, "\n");
          }
        } 


        
        if ((nframes%100) == 0 || rx_gain_offset == 0) {
          if (srslte_rf_has_rssi(&rf)) {
            rx_gain_offset = 10*log10(rssi*1000)-srslte_rf_get_rssi(&rf);
          } else {
            rx_gain_offset = srslte_rf_get_rx_gain(&rf);            
          }
        }
        
        // Plot and Printf
        if ((nframes%10) == 0) {

          printf("CFO: %+8.4f kHz, SFO: %+8.4f Hz, RSSI: %5.1f dBm, RSSI/ref-symbol: %+5.1f dBm, "
                 "RSRP: %+5.1f dBm, RSRQ: %5.1f dB, SNR: %5.1f dB\r",
                srslte_ue_sync_get_cfo(&ue_sync)/1000, srslte_ue_sync_get_sfo(&ue_sync), 
                10*log10(rssi*1000) - rx_gain_offset,                        
                10*log10(rssi_utra*1000)- rx_gain_offset, 
                10*log10(rsrp*1000) - rx_gain_offset, 
                10*log10(rsrq), 10*log10(snr));                
          if (srslte_verbose != SRSLTE_VERBOSE_NONE) {
            printf("\n");
          }
        }
        break;
      }
      if (srslte_ue_sync_get_sfidx(&ue_sync) == 9) {
        sfn++; 
        if (sfn == 1024) {
          sfn = 0; 
        }
      }
    } else if (ret == 0) {
      printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r", 
        srslte_sync_get_peak_value(&ue_sync.sfind), 
        ue_sync.frame_total_cnt, ue_sync.state);      
    }
   
        
    sf_cnt++;                  
  } // Main loop

  for (int i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
    if (data[i]) {
      free(data[i]);
    }
  }

  if(crs)
    fclose(crs);

  srslte_ue_sync_free(&ue_sync);
  srslte_rf_close(&rf);
  printf("\nBye\n");
  exit(0);
}


static uint32 
cell_measurement_decode_sib(uint8_t * data, uint32_t n)
{
  uint32 sib_type_bmp = 0;

  printf("SI Decode, bits=%d\n", n);
  srslte::bit_buffer_t bit_buf;
  LIBLTE_RRC_BCCH_DLSCH_MSG_STRUCT dlsch_msg;

  srslte_bit_unpack_vector(data, bit_buf.msg, n);
  bit_buf.N_bits = n;

  LIBLTE_ERROR_ENUM e = liblte_rrc_unpack_bcch_dlsch_msg((LIBLTE_BIT_MSG_STRUCT *) &bit_buf, &dlsch_msg);
  
  if(e == LIBLTE_SUCCESS)
  {
    for(uint32 i = 0; i < dlsch_msg.N_sibs; ++i)
    {
      sib_type_bmp |= 1 << (uint32)dlsch_msg.sibs[i].sib_type;
      
      switch(dlsch_msg.sibs[i].sib_type)
      {
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1:
          cell_measurement_decode_sib1(&dlsch_msg.sibs[i].sib.sib1);
          break;
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2:
          cell_measurement_decode_sib2(&dlsch_msg.sibs[i].sib.sib2);
          break;
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_3:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_4:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_5:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_6:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_7:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_8:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_9:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_10:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_11:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_12:
        case LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_13:
          printf("WARNING: Unsupported SIB type %s\n", liblte_rrc_sib_type_text[dlsch_msg.sibs[i].sib_type]);
          break;
    
        default:
          printf("ERROR: Unknown SIB type %s\n", liblte_rrc_sib_type_text[dlsch_msg.sibs[i].sib_type]);
          exit(1);
      }
    }
  }
  else
  {
    printf("Error Decoding SIB %d\n", e);
    exit(1);
  }
  

  return (uint32) sib_type_bmp;
}


static void
cell_measurement_decode_sib1(const LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_1_STRUCT * sib1)
{
  printf("----------------------------------------------------------\n");
  printf("SIB1 received: \n");

  // PLMNs
  for(uint32 i = 0; i < sib1->N_plmn_ids; ++i)
    printf("PLMN: MCC=%03X MNC=%03X\n", sib1->plmn_id[i].id.mcc, sib1->plmn_id[i].id.mnc);

  // Scheduling Info
  printf("Scheduling Info (%d):\n", sib1->N_sched_info);

  for (uint32 i = 0; i < sib1->N_sched_info; ++i)
  {
    if (i == 0)
    {
      printf("SIB2: Periodicity=%s\n", liblte_rrc_si_periodicity_text[sib1->sched_info[i].si_periodicity]);	
    }
    else
    {
      printf("SIBS: [ ");

      for(uint32 j = 0; j < sib1->sched_info[i].N_sib_mapping_info; ++j)
        printf("%s", liblte_rrc_sib_type_text[sib1->sched_info[i].sib_mapping_info[j].sib_type]);

      printf(" ] Periodicity=%s\n", liblte_rrc_si_periodicity_text[sib1->sched_info[i].si_periodicity]);
    }
  }

  // TDD
  if(sib1->tdd)
    printf("TDD Info: [SF Assignment=%d Special SF Patterns=%d]\n", sib1->tdd_cnfg.sf_assignment, sib1->tdd_cnfg.special_sf_patterns);
  else
    printf("TDD Info: []\n");

  printf("Cell ID=%d TAC=%d QRXLEV_min=%d QRXLEV_min_offset=%d SI-ValueTag=%d FreqBand=%d\n", 
    sib1->cell_id, sib1->tracking_area_code, sib1->q_rx_lev_min, sib1->q_rx_lev_min_offset, sib1->system_info_value_tag, sib1->freq_band_indicator);
  printf("Cell Barring: [%s]\n", liblte_rrc_cell_barred_text[sib1->cell_barred]);
  printf("Intra-Freq Reselection: [%s]\n", liblte_rrc_intra_freq_reselection_text[sib1->intra_freq_reselection]);
  printf("SI Window Length: [%s]\n", liblte_rrc_si_window_length_text[sib1->si_window_length]);
  printf("CSG Indicator=%d, CSG Id=%d\n", sib1->csg_indication, sib1->csg_id);
  printf("----------------------------------------------------------\n");
}


static void
cell_measurement_decode_sib2(const LIBLTE_RRC_SYS_INFO_BLOCK_TYPE_2_STRUCT * sib2)
{
  printf("----------------------------------------------------------\n");
  printf("SIB2 received: \n");
  printf("Barring: Present=%d AC Emergency=%d MO Data=%d MO Signalling=%d\n", sib2->ac_barring_info_present, sib2->ac_barring_for_emergency, sib2->ac_barring_for_mo_data.enabled, sib2->ac_barring_for_mo_signalling.enabled);

  if(sib2->ac_barring_for_mo_data.enabled)
    printf("MO Data: Special AC:%d, Time=%ss Factor=%s\n",sib2->ac_barring_for_mo_data.for_special_ac, liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_data.time], liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_data.factor]);

  if(sib2->ac_barring_for_mo_signalling.enabled)
    printf("MO Signalling: Special AC:%d, Time=%ss Factor=%s\n",sib2->ac_barring_for_mo_data.for_special_ac, liblte_rrc_ac_barring_time_text[sib2->ac_barring_for_mo_data.time], liblte_rrc_ac_barring_factor_text[sib2->ac_barring_for_mo_data.factor]);


  if(sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.present)
  {
    printf("RACH: Group A: SizeRA=%s MsgSize=%s PwrOffsetGroupB=%s\n", 
      liblte_rrc_size_of_ra_preambles_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.size_of_ra],
      liblte_rrc_message_size_group_a_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_size],
      liblte_rrc_message_power_offset_group_b_text[sib2->rr_config_common_sib.rach_cnfg.preambles_group_a_cnfg.msg_pwr_offset_group_b] );
  }

  printf("RACH: NumRAPreambles=%s RampStep=%s InitalRxTargetPwr=%s TransMax=%s ResponseWindow=%s CrTimer=%s MaxHarqMsg3=%d\n",
    liblte_rrc_number_of_ra_preambles_text[sib2->rr_config_common_sib.rach_cnfg.num_ra_preambles],
    liblte_rrc_power_ramping_step_text[sib2->rr_config_common_sib.rach_cnfg.pwr_ramping_step],
    liblte_rrc_preamble_initial_received_target_power_text[sib2->rr_config_common_sib.rach_cnfg.preamble_init_rx_target_pwr],
    liblte_rrc_preamble_trans_max_text[sib2->rr_config_common_sib.rach_cnfg.preamble_trans_max],
    liblte_rrc_ra_response_window_size_text[sib2->rr_config_common_sib.rach_cnfg.ra_resp_win_size],
    liblte_rrc_mac_contention_resolution_timer_text[sib2->rr_config_common_sib.rach_cnfg.mac_con_res_timer],
    sib2->rr_config_common_sib.rach_cnfg.max_harq_msg3_tx);

  printf("BCCH: ModPeriodCoeff=%s\n", liblte_rrc_modification_period_coeff_text[sib2->rr_config_common_sib.bcch_cnfg.modification_period_coeff]);
  printf("PCCH: DefaultPagingCycle=%s nB=%s\n",
    liblte_rrc_default_paging_cycle_text[sib2->rr_config_common_sib.pcch_cnfg.default_paging_cycle],
    liblte_rrc_nb_text[sib2->rr_config_common_sib.pcch_cnfg.nB]);

  printf("PRACH: ConfigIndex=%d ZeroCorrelationZone=%d FreqOffset=%d HighSpeedFlag=%d RootSeqIndex=%d\n",
    sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_config_index,
    sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.zero_correlation_zone_config,
    sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.prach_freq_offset,
    sib2->rr_config_common_sib.prach_cnfg.prach_cnfg_info.high_speed_flag,
    sib2->rr_config_common_sib.prach_cnfg.root_sequence_index);

  printf("PDSCH: Pb=%d RsPower=%ddBm\n",
    sib2->rr_config_common_sib.pdsch_cnfg.p_b,
    sib2->rr_config_common_sib.pdsch_cnfg.rs_power);

  printf("PUSCH: RsGrpAssignmt=%d RsCs=%d RsGroupHop=%d RsSeqHop=%d HopMode=%d Nsb=%d HopOffset=%d 64QAM=%d",
    sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_assignment_pusch,
    sib2->rr_config_common_sib.pusch_cnfg.ul_rs.cyclic_shift,
    sib2->rr_config_common_sib.pusch_cnfg.ul_rs.group_hopping_enabled,
    sib2->rr_config_common_sib.pusch_cnfg.ul_rs.sequence_hopping_enabled,
    sib2->rr_config_common_sib.pusch_cnfg.hopping_mode,
    sib2->rr_config_common_sib.pusch_cnfg.n_sb,
    sib2->rr_config_common_sib.pusch_cnfg.pusch_hopping_offset,
    sib2->rr_config_common_sib.pusch_cnfg.enable_64_qam);

  printf("PUCCH: DeltaPucchShift=%s N1PucchAn=%d NrbCQI=%d NCsAn=%d\n",
    liblte_rrc_delta_pucch_shift_text[sib2->rr_config_common_sib.pucch_cnfg.delta_pucch_shift],
    sib2->rr_config_common_sib.pucch_cnfg.n1_pucch_an,
    sib2->rr_config_common_sib.pucch_cnfg.n_rb_cqi,
    sib2->rr_config_common_sib.pucch_cnfg.n_cs_an);

  if(sib2->rr_config_common_sib.srs_ul_cnfg.present)
  {
    printf("SRS: BwConfig=%s SfConfig=%s AckNackSimulTx=%d\n", 
      liblte_rrc_srs_bw_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.bw_cnfg],
      liblte_rrc_srs_subfr_config_text[sib2->rr_config_common_sib.srs_ul_cnfg.subfr_cnfg],
      sib2->rr_config_common_sib.srs_ul_cnfg.ack_nack_simul_tx
    );

    if(sib2->rr_config_common_sib.srs_ul_cnfg.max_up_pts_present)
        printf("SRS: MaxUpPts=%d\n", sib2->rr_config_common_sib.srs_ul_cnfg.max_up_pts);
  }

  printf("ULPWR: DeltaFlist F1=%s F1b=%s F2=%s F2a=%s F2b=%s\n",
    liblte_rrc_delta_f_pucch_format_1_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1],
    liblte_rrc_delta_f_pucch_format_1b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_1b],
    liblte_rrc_delta_f_pucch_format_2_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2],
    liblte_rrc_delta_f_pucch_format_2a_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2a],
    liblte_rrc_delta_f_pucch_format_2b_text[sib2->rr_config_common_sib.ul_pwr_ctrl.delta_flist_pucch.format_2b]);

  printf("ULPWR: Alpha=%s P0NomPusch=%d P0NomPucch=%d DeltaPreambleMsg3=%d\n",
    liblte_rrc_ul_power_control_alpha_text[sib2->rr_config_common_sib.ul_pwr_ctrl.alpha],
    sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pusch,
    sib2->rr_config_common_sib.ul_pwr_ctrl.p0_nominal_pucch,
    sib2->rr_config_common_sib.ul_pwr_ctrl.delta_preamble_msg3);

  printf("ULCP: Length=%s\n", liblte_rrc_ul_cp_length_text[sib2->rr_config_common_sib.ul_cp_length]);

  printf("T&C: T300=%s T301=%s T310=%s N310=%s T311=%s N311=%s\n",
  liblte_rrc_t300_text[sib2->ue_timers_and_constants.t300],
    liblte_rrc_t301_text[sib2->ue_timers_and_constants.t301],
    liblte_rrc_t310_text[sib2->ue_timers_and_constants.t310],
    liblte_rrc_n310_text[sib2->ue_timers_and_constants.n310],
    liblte_rrc_t311_text[sib2->ue_timers_and_constants.t311],
    liblte_rrc_n311_text[sib2->ue_timers_and_constants.n311] );

  printf("Additional Spectrum Emission=%d MBSFN Config Size=%d\n", sib2->additional_spectrum_emission, sib2->mbsfn_subfr_cnfg_list_size);
  printf("TAT=%s\n", liblte_rrc_time_alignment_timer_text[sib2->time_alignment_timer]);
  printf("UL Bandwidth Present=%d Value=%s\n", sib2->ul_bw.present, liblte_rrc_ul_bw_text[sib2->ul_bw.bw]);
  printf("EARFCN Present=%d Value=%d\n", sib2->arfcn_value_eutra.present, sib2->arfcn_value_eutra.value);

  printf("----------------------------------------------------------\n");
}



