#!/bin/awk

### This script currently prints the total number of rows processed.
### You must edit this script to print the average of the 2nd column
### instead of the number of rows.

# This block of code is executed for each line in the file
{
    sum+=$6
    # The script should NOT print out a value for each line
}
# The END block is processed after the last line is read
END {
    # NR is a variable equal to the number of rows in the file
    print "Average: " sum/ NR
    # Change this to print the Average instead of just the number of rows
}
