#!/usr/bin/env node

// Playground to experiment with truncatec mean calculations

/* eslint-disable no-console */

const data = [
  2050,
  2055,
  2048,
  2,
  2051,
  2058,
  2050,
  3000
];

const win = 1.1;

function truncated_mean_3pass() {
  let count = data.length;

  // Collect mean
  let s_mean = 0;
  for (let i = 0; i < count; i++) {
    s_mean += data[i];
  }
  let mean = Math.round(s_mean / count);

  // Collect sigma
  let s_sigma = 0;
  for (let i = 0; i < count; i++) {
    let val = data[i];
    s_sigma += (mean - val) * (mean - val);
  }

  let sigma_square_4 = Math.round(s_sigma * win * win / count);
  let sigma = Math.round(Math.sqrt(s_sigma / count));

  // Drop big deviations and count mean for the rest
  let s_mean_filtered = 0;
  let s_mean_filtered_cnt = 0;
  for (let i = 0; i < count; i++) {
    let val = data[i];

    if ((mean - val) * (mean - val) < sigma_square_4) {
      s_mean_filtered += val;
      s_mean_filtered_cnt++;
    }
  }

  let truncated_mean = mean;

  if (s_mean_filtered_cnt) {
    truncated_mean = Math.round(s_mean_filtered / s_mean_filtered_cnt);
  }

  console.log('3 Passes');
  console.log('');
  console.log(`Mean:   ${mean}`);
  console.log(`Result: ${truncated_mean}`);
  console.log(`Sigma:  ${sigma} (allow [${Math.round(mean - win * sigma)}..${Math.round(mean + win * sigma)}])`);
  console.log(`Win:    ${win}`);
  console.log(`Used:   ${s_mean_filtered_cnt} of ${count}`);
  console.log('');
}


function truncated_mean_2pass() {
  let count = data.length;

  let s = 0;
  let s2 = 0;
  for (let i = 0; i < count; i++) {
    let val = data[i];
    s += val;
    s2 += val * val;
  }
  let mean = Math.round(s / count);

  let sigma2 = (s2 - (s * s / count)) / count;

  let sigma_square_4 = Math.round(sigma2 * win * win);
  let sigma = Math.round(Math.sqrt(sigma2));

  // Drop big deviations and count mean for the rest
  let s_mean_filtered = 0;
  let s_mean_filtered_cnt = 0;
  for (let i = 0; i < count; i++) {
    let val = data[i];

    if ((mean - val) * (mean - val) < sigma_square_4) {
      s_mean_filtered += val;
      s_mean_filtered_cnt++;
    }
  }

  let truncated_mean = mean;

  if (s_mean_filtered_cnt) {
    truncated_mean = Math.round(s_mean_filtered / s_mean_filtered_cnt);
  }

  console.log('2 Passes');
  console.log('');
  console.log(`Mean:   ${mean}`);
  console.log(`Result: ${truncated_mean}`);
  console.log(`Sigma:  ${sigma} (allow [${Math.round(mean - win * sigma)}..${Math.round(mean + win * sigma)}])`);
  console.log(`Win:    ${win}`);
  console.log(`Used:   ${s_mean_filtered_cnt} of ${count}`);
  console.log('');
}



console.log('');
console.log(`Data: ${data}`);
console.log('');

truncated_mean_3pass();
truncated_mean_2pass();
