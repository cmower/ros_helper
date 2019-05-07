from sensor_msgs.msg import JointState, Joy
from ros_helper.msg import MultiJoy, Keyboard

__all__=['JointStateMsg', 'JoyMsg', 'MultiJoyMsg', 'Keyboard']

# Keyboard keys, ord : pygame key id
keyboard_keys = {\
                 48 : 'K_0',\
                 49 : 'K_1',\
                 50 : 'K_2',\
                 51 : 'K_3',\
                 52 : 'K_4',\
                 53 : 'K_5',\
                 54 : 'K_6',\
                 55 : 'K_7',\
                 56 : 'K_8',\
                 57 : 'K_9',\
                 38 : 'K_AMPERSAND',\
                 42 : 'K_ASTERISK',\
                 64 : 'K_AT',\
                 96 : 'K_BACKQUOTE',\
                 92 : 'K_BACKSLASH',\
                 8 : 'K_BACKSPACE',\
                 318 : 'K_BREAK',\
                 301 : 'K_CAPSLOCK',\
                 94 : 'K_CARET',\
                 12 : 'K_CLEAR',\
                 58 : 'K_COLON',\
                 44 : 'K_COMMA',\
                 127 : 'K_DELETE',\
                 36 : 'K_DOLLAR',\
                 274 : 'K_DOWN',\
                 279 : 'K_END',\
                 61 : 'K_EQUALS',\
                 27 : 'K_ESCAPE',\
                 321 : 'K_EURO',\
                 33 : 'K_EXCLAIM',\
                 282 : 'K_F1',\
                 291 : 'K_F10',\
                 292 : 'K_F11',\
                 293 : 'K_F12',\
                 294 : 'K_F13',\
                 295 : 'K_F14',\
                 296 : 'K_F15',\
                 283 : 'K_F2',\
                 284 : 'K_F3',\
                 285 : 'K_F4',\
                 286 : 'K_F5',\
                 287 : 'K_F6',\
                 288 : 'K_F7',\
                 289 : 'K_F8',\
                 290 : 'K_F9',\
                 0 : 'K_FIRST',\
                 62 : 'K_GREATER',\
                 35 : 'K_HASH',\
                 315 : 'K_HELP',\
                 278 : 'K_HOME',\
                 277 : 'K_INSERT',\
                 256 : 'K_KP0',\
                 257 : 'K_KP1',\
                 258 : 'K_KP2',\
                 259 : 'K_KP3',\
                 260 : 'K_KP4',\
                 261 : 'K_KP5',\
                 262 : 'K_KP6',\
                 263 : 'K_KP7',\
                 264 : 'K_KP8',\
                 265 : 'K_KP9',\
                 267 : 'K_KP_DIVIDE',\
                 271 : 'K_KP_ENTER',\
                 272 : 'K_KP_EQUALS',\
                 269 : 'K_KP_MINUS',\
                 268 : 'K_KP_MULTIPLY',\
                 266 : 'K_KP_PERIOD',\
                 270 : 'K_KP_PLUS',\
                 308 : 'K_LALT',\
                 323 : 'K_LAST',\
                 306 : 'K_LCTRL',\
                 276 : 'K_LEFT',\
                 91 : 'K_LEFTBRACKET',\
                 40 : 'K_LEFTPAREN',\
                 60 : 'K_LESS',\
                 310 : 'K_LMETA',\
                 304 : 'K_LSHIFT',\
                 311 : 'K_LSUPER',\
                 319 : 'K_MENU',\
                 45 : 'K_MINUS',\
                 313 : 'K_MODE',\
                 300 : 'K_NUMLOCK',\
                 281 : 'K_PAGEDOWN',\
                 280 : 'K_PAGEUP',\
                 19 : 'K_PAUSE',\
                 46 : 'K_PERIOD',\
                 43 : 'K_PLUS',\
                 320 : 'K_POWER',\
                 316 : 'K_PRINT',\
                 63 : 'K_QUESTION',\
                 39 : 'K_QUOTE',\
                 34 : 'K_QUOTEDBL',\
                 307 : 'K_RALT',\
                 305 : 'K_RCTRL',\
                 13 : 'K_RETURN',\
                 275 : 'K_RIGHT',\
                 93 : 'K_RIGHTBRACKET',\
                 41 : 'K_RIGHTPAREN',\
                 309 : 'K_RMETA',\
                 303 : 'K_RSHIFT',\
                 312 : 'K_RSUPER',\
                 302 : 'K_SCROLLOCK',\
                 59 : 'K_SEMICOLON',\
                 47 : 'K_SLASH',\
                 32 : 'K_SPACE',\
                 317 : 'K_SYSREQ',\
                 9 : 'K_TAB',\
                 95 : 'K_UNDERSCORE',\
                 0 : 'K_UNKNOWN',\
                 273 : 'K_UP',\
                 97 : 'K_a',\
                 98 : 'K_b',\
                 99 : 'K_c',\
                 100 : 'K_d',\
                 101 : 'K_e',\
                 102 : 'K_f',\
                 103 : 'K_g',\
                 104 : 'K_h',\
                 105 : 'K_i',\
                 106 : 'K_j',\
                 107 : 'K_k',\
                 108 : 'K_l',\
                 109 : 'K_m',\
                 110 : 'K_n',\
                 111 : 'K_o',\
                 112 : 'K_p',\
                 113 : 'K_q',\
                 114 : 'K_r',\
                 115 : 'K_s',\
                 116 : 'K_t',\
                 117 : 'K_u',\
                 118 : 'K_v',\
                 119 : 'K_w',\
                 120 : 'K_x',\
                 121 : 'K_y',\
                 122 : 'K_z'\
}

class JointStateMsg(JointState):

    def __init__(self, t, q, qd=None, eff=None, names=None):
        super(JointStateMsg, self).__init__()
        self.header.stamp = t
        self.position = q
        if names is not None: self.name = names
        if qd is not None: self.velocity = qd
        if eff is not None: self.effort = eff

class JoyMsg(Joy):

    def __init__(self, t, axes, buttons):
        super(JoyMsg, self).__init__()
        self.header.stamp = t
        self.axes = axes
        self.buttons = buttons

class MultiJoyMsg(MultiJoy):

    def __init__(self, t, joys):
        super(MultiJoyMsg, self).__init__()
        self.header.stamp = t
        self.njoys = len(joys)
        self.joys = joys
        
class KeyboardMsg(Keyboard):

    def __init__(self, t):
        super(KeyboardMsg, self).__init__()
        self.add_time(t)

    def add_time(self, t):
        self.header.stamp = t

    def __setitem__(self, idx, val):
        setattr(self, KeyboardMsg._idx_to_key(idx), val)

    def __getitem__(self, idx):
        return getattr(self, KeyboardMsg._idx_to_key(idx))

    @staticmethod
    def _idx_to_key(idx):
        if type(idx) is int:
            k = keyboard_keys[idx]
        elif type(idx) is str:
            k = idx
        else:
            raise "[ERROR] given idx can only be a int or str."
        return k    

    @staticmethod
    def to_keyboardmsg(kb):
        kbmsg = KeyboardMsg()
        for idx in keyboard_keys:
            k = KeyboardMsg._idx_to_key(idx)
            kbmsg[k] = getattr(kb, k)
        return kbmsg
