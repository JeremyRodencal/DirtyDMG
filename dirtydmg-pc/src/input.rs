use sdl2::joystick::HatState;

#[derive(Copy, Clone)]
struct DpadStates
{
    pub button_up: bool,
    pub button_down: bool,
    pub button_left: bool,
    pub button_right: bool
}

impl DpadStates{
    fn new() -> DpadStates
    {
        DpadStates{
            button_up: false,
            button_down: false,
            button_left: false,
            button_right: false
        }
    }

    fn set_states(&mut self, hat_state: sdl2::joystick::HatState)
    {
        self.button_up = false;
        self.button_down = false;
        self.button_left = false;
        self.button_right = false;
        match hat_state
        {
            HatState::Centered => { },
            HatState::Up => {
                self.button_up = true;
            },
            HatState::LeftUp => {
                self.button_left = true;
                self.button_up = true;
            },
            HatState::Left => {
                self.button_left = true;
            },
            HatState::LeftDown => {
                self.button_down = true;
                self.button_left = true;
            },
            HatState::Down => {
                self.button_down = true;
            },
            HatState::RightDown => {
                self.button_down = true;
                self.button_right = true;
            },
            HatState::Right => {
                self.button_right = true;
            },
            HatState::RightUp => {
                self.button_up = true;
                self.button_right = true;
            }
        }
    }
}

pub struct HatSwitchHandler
{
    current_states: DpadStates,
    last_states: DpadStates,
}

impl HatSwitchHandler 
{
    pub fn new() -> HatSwitchHandler
    {
        HatSwitchHandler { 
            current_states: DpadStates::new(),
            last_states: DpadStates::new(),
        }
    }

    pub fn update(&mut self, hat_state: sdl2::joystick::HatState)
    {
        self.last_states = self.current_states;
        self.current_states.set_states(hat_state);
    }

    pub fn up_changed(&self) -> Option<bool>
    {
        if self.last_states.button_up != self.current_states.button_up { 
            Some(self.current_states.button_up)
        }
        else {None}
    }

    pub fn down_changed(&self) -> Option<bool>
    {
        if self.last_states.button_down != self.current_states.button_down { 
            Some(self.current_states.button_down)
        }
        else {None}
    }

    pub fn left_changed(&self) -> Option<bool>
    {
        if self.last_states.button_left != self.current_states.button_left {
            Some(self.current_states.button_left)
        }
        else {None}
    }

    pub fn right_changed(&self) -> Option<bool>
    {
        if self.last_states.button_right != self.current_states.button_right {
            Some(self.current_states.button_right)
        }
        else {None}
    }


}
