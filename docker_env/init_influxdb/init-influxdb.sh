#!/bin/bash
set -e

influx -execute "CREATE DATABASE power_monitor"
influx -execute "CREATE RETENTION POLICY \"one_year\" ON \"power_monitor\" DURATION 52w REPLICATION 1 DEFAULT"
