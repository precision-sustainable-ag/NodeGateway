**Gateway &amp; Node**

- Remove sending sensor IDs to Gateway --> DONE 17-Jan-2020
  - Already done in MOD-(May\_19) branch (Node\_Acclima\_v17\_13-Jul-2019.ino)
- Periodic sync checks
  - Daily?
  - Attach to NIST update on Gateway?
- Remove siteID variable, use serial number as identifier instead --> DONE 22-Jan-2020
- Tweak radio settings?
- Hard code radio IDs based off SN with option to edit (in case someone has two units whose last two digits are the same) --> DONE 22-Jan-2020
- Add Signal Strength to data string   lastRssi() returns the signal strength --> DONE 22-Jan-2020

**Gateway**

- Add check to NIST update to (check timestamp is correct) --> DONE 17-Jan-2020 
  - If year < 2016, timestamp is bad, if year > 2050
  - Can check if month > 12
  - Reject bad timestamps from NIST
- Remove NISThr &amp; NISTmin variables ? hardcode a time (can be used to verify hr value extracted from NIST response --> DONE 22-Jan-2020
- Save power data and SS data to SD --> DONE 22-Jan-2020
- HOLD - Save error log to SD (errors should be timestamped, save to data file instead?) - is it worth the effort?
  - What to save?
    - Missed node IDs (after sync attempt)
    - Failure to connect to Hologram (define an error code?)
    - Failure to connect to NIST server
    - When battV \&lt; minimum allowed
    - When synchronization initiated (to indicate if G reset)
    - Missed LoRa transmission from a node
    - Missing data from a sensor (Node sends error?)

**Node**

- Scan through Flash on startup to find next address to write to ? code on Github
- How to check for bad timestamp? --> DONE 22-Jan-2020
  - If year < 2016, timestamp is bad, if year > 2050
  - Can check if month > 12
  - Reject bad timestamps from Gateway --> NO!!! It'll mess up synchronization
  - NEED TO FIGURE OUT: What to do if Node receives bad timestamp? Do nothing?
- Decided: one long data string is better than multiple shorter strings (don&#39;t send separate strings for individual sensors)
