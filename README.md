# beatcube

Note: Requires CMSIS v 5.3.0 or higher with real_fast_f32 FFT function.

See thread located here for information on upgrading teensy's arm_math to include the proper library:
		https://forum.pjrc.com/threads/53809-CMSIS-NN-(Neural-Network)-Library-Now-Working-on-Teensy-3-6?p=188532#post188532
		
		Notes:
			Ignore step #1.
			Download the correct 5.3 version (they changed a lot after 5.3 and I can't figure out how to update, but FFT hasn't changed)
				https://github.com/ARM-software/CMSIS_5/releases/tag/5.3.0
			There may have been a couple more files I needed from the 5.3 download, make use of the arduino debugging output to copy them over.	
			