# Lock with a doorbell

This accessory is made for a "Access control system" in an apartment. aka "Building block system". Just like this:

![Building block system](https://ipic.vizards.cc/h8kxi.jpg)

## Functions

Accessory | Function  | Instructions
:--: | :--: | :--: 
Lock | Unlock & Auto lock | Lock automatically after 3 seconds of unlocking.
Doorbell | Send a notification when someone rang the doorbell | Must be used in conjunction with the camera<sup>[[1]](#fn1)</sup>.
Switch | Enable/Disable Auto UnLock | When the switch is on, lock will automatically unlock when someone rings the doorbell.

## Circuit Explanation

- Use a MOSFET/triode to implement the switch circuit. When `lock_gpio` is high_level, A SIG pin on the ACS circuit board will connect to GND through a MOSFET/triode. the lock will unlock.
- The doorbell will ring when the voltage between the speakers and GND is 1.1V. I will check the doorbell voltage at `doorbell_gpio` pin, filter it and send a notification.

---

<ol>
<li name="fn1">Doorbells are not allowed to run separately, It must be used as an accessory to the camera. So I send a doorbell notification to the camera through HTTP request. Doorbell will not displayed as an accessory in the Home App.</li>
</ol>