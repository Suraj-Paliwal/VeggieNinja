"""Human-in-the-loop settings for the okra pick. Tune BASELINE_CONF once the fine-tuned detector exists."""

# When to ask the operator
MODE = "uncertain"          # "always": ask every time | "uncertain": ask only when unsure | "never": auto only
BASELINE_CONF = 0.60        # median detector confidence needed to act without asking
CONFUSION_GAP = 0.15        # a second candidate within this confidence of the best = confusion -> ask
MIN_SEEN_FRAC = 0.6         # candidate must be seen in >= 60 % of the frames to act without asking
MIN_FILTER_OK = 0.8         # sanity filter must accept it in >= 80 % of its sightings
MAX_SPREAD_M = 0.015        # 3D position must agree within 1.5 cm across frames
SHOW_FLOOR = 0.15           # candidates below this median confidence are not shown to the operator
ASK_WHEN_NOTHING = True     # no candidate at all -> ask "is there an okra I missed?" (finds detector misses)

# Robot feedback while asking (through robot_agent; ignored if the agent is not running)
SPEAK = True
SAY_ASK = "I found something. Is this an okra? Please check the screen."
SAY_NOTHING = "I do not see any okra. Is there one?"
SAY_YES = "Thank you. Picking it."
SAY_NO = "Okay. I will not pick it."
LED_ASK, LED_YES, LED_NO = (255, 160, 0), (0, 255, 0), (255, 0, 0)

# Where everything is stored (one folder per event)
DATA_ROOT = "~/Junction/okra_data"
