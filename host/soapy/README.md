# SoapySDR module

The W1700K's SoapySDR module (`driver=w1700k`, for SDR++, GNU Radio and anything else on SoapySDR)
lives in its own repository: <https://github.com/kingjamez/SoapyW1700K>.

It reads the IQ from [`w1700k-stream`](../stream/README.md) and controls the radio through
[`host/bin/w1700k`](../w1700k/README.md), so it needs this project running. It needs the stream
service's `rf.lo_ranges_hz` (the span each band locks over), which `w1700k status --rf` reports.
