#include <iostream>

#include "controller.hh"
#include "timestamp.hh"
#include <math.h>
#include <algorithm>

using namespace std;

// METHOD NUM:
// 1 Default
// 2 AIMD
// 3 Delay-triggered scheme
// 4 - 7 contest models (METHOD 7 is the final model)
#define METHOD 7

// Globals
double timeouts_thresh = 0;
unsigned int num_timeouts = 0;
double the_window_size = 14.0;
bool slow_start = true;
double rto = 80;
double estimate_rtt = 80;
double phi = 4;
double sigma = 0.2;
double deviation = 0.0;
unsigned int ssthresh = 999999;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug )
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  /* Default: fixed window size of 100 outstanding datagrams */

  if ( debug_ ) {
  cerr << "At time " << timestamp_ms()
   << " window size is " << the_window_size << endl;
  }
  if (the_window_size < 1) the_window_size = 1.0;
  return (unsigned int) the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
          /* of the sent datagram */
          const uint64_t send_timestamp )
                  /* in milliseconds */
{
  /* Default: take no action */

  if ( debug_ ) {
  cerr << "At time " << send_timestamp
   << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
           /* what sequence number was acknowledged */
           const uint64_t send_timestamp_acked,
           /* when the acknowledged datagram was sent (sender's clock) */
           const uint64_t recv_timestamp_acked,
           /* when the acknowledged datagram was received (receiver's clock)*/
           const uint64_t timestamp_ack_received )
                 /* when the ack was received (by sender) */
{
  /* Default: take no action */

  if ( debug_ ) {
  cerr << "At time " << timestamp_ack_received
   << " received ack for datagram " << sequence_number_acked
   << " (send @ time " << send_timestamp_acked
   << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
   << endl;
  }

  // AIMD
  #if METHOD == 2
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt <= timeout_ms()) {
     the_window_size += 1.0/the_window_size;
    } else {
     the_window_size /= 2;
     if (the_window_size < 1) the_window_size = 1;
    }
  #endif

  // Delay-triggered scheme
  #if METHOD == 3
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    uint64_t upper = 100;
    uint64_t lower = 50;
    if (rtt > upper) {
      the_window_size -= 1.0/the_window_size;
      if (the_window_size < 1) the_window_size = 1;
    }
    if (rtt <= lower) the_window_size += 1.0/the_window_size;
  #endif

  // Slow start + AIMD
  // Average capacity: 5.04 Mbits/s
  // Average throughput: 1.96 Mbits/s (38.9% utilization)
  // 95th percentile per-packet queueing delay: 37 ms
  // 95th percentile signal delay: 90 ms
  #if METHOD == 4
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt > timeout_ms()) {
      slow_start = false;
      the_window_size /= 2; //decrease window size by half
    } else {
      if (slow_start) { //slow start
        the_window_size += 1;
      } else {
        the_window_size += 1.0/the_window_size;
      }
    }
  #endif


  // Slow start + AIMD
  // Average capacity: 5.04 Mbits/s
  // Average throughput: 2.65 Mbits/s (52.6% utilization)
  // 95th percentile per-packet queueing delay: 49 ms
  // 95th percentile signal delay: 99 ms
  #if METHOD == 5
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt > timeout_ms()) {
      ssthresh = the_window_size / 2;
      the_window_size = 1;  //decrease window size to 1
    } else {
      if (the_window_size < ssthresh) { //slow start
        the_window_size += 1;
      } else {
        the_window_size += 1.0/the_window_size;
      }
    }
  #endif


  // Slow start + AIMD + variant RTO
  // Average capacity: 5.04 Mbits/s
  // Average throughput: 1.63 Mbits/s (32.3% utilization)
  // 95th percentile per-packet queueing delay: 62 ms
  // 95th percentile signal delay: 158 ms
  #if METHOD == 6
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt > timeout_ms()) {
      ssthresh = the_window_size / 2;
      the_window_size = 1;
    } else {
      if (the_window_size < ssthresh) { //slow start
        the_window_size += 1;
      } else {
        the_window_size += 1.0/the_window_size;
      }
    }
    double diff = 1.0 * rtt - 1.0 * estimate_rtt;
    estimate_rtt = estimate_rtt + (sigma * (diff));
    deviation = deviation + 1.0 * sigma * (fabs(diff) - deviation);
    rto = estimate_rtt + phi * deviation;
  #endif


  // Final Model
  // Average capacity: 5.04 Mbits/s
  // Average throughput: 3.60 Mbits/s (71.3% utilization)
  // 95th percentile per-packet queueing delay: 56 ms
  // 95th percentile signal delay: 100 ms
  #if METHOD == 7
    uint64_t rtt = timestamp_ack_received - send_timestamp_acked;
    if (rtt > timeout_ms()) {
      slow_start = false; //exit slow start stage
      if (num_timeouts >= timeouts_thresh) {
        num_timeouts = 0;
        timeouts_thresh = the_window_size;
        the_window_size /= 2.0;
      } else {
        num_timeouts += 1;
      }
    } else if (slow_start) {
      the_window_size += 2; //slow start
    } else { //congestion avoidance
      the_window_size += 1.5 / the_window_size;
    }
  #endif
}


/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  #if METHOD == 3 // delay-triggered
    return 50;
  #endif

  #if METHOD == 4 || METHOD == 5 || METHOD == 6 || METHOD == 7 // contest
    return (unsigned int) rto;
  #endif

  return 1000; // default
}
