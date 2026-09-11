from switch_config import switch_cfg_by_target

class Switch:
    TYPE_2POS = "2POS"
    TYPE_3POS = "3POS"
    TYPE_ADC = "ADC"

    def __init__(self, name, sw_type, flags):
        self.name = name
        self.type = sw_type
        self.flags = flags
        self.inverted = False
        self.default = None
        self.display = None


class Switch2POS(Switch):
    def __init__(self, name, gpio, pin, flags=0):
        super(Switch2POS, self).__init__(name, Switch.TYPE_2POS, flags)
        self.gpio = gpio
        self.pin = pin


class Switch3POS(Switch):
    def __init__(self, name, gpio_high, pin_high, gpio_low, pin_low, flags=0):
        super(Switch3POS, self).__init__(name, Switch.TYPE_3POS, flags)
        self.gpio_high = gpio_high
        self.pin_high = pin_high
        self.gpio_low = gpio_low
        self.pin_low = pin_low


class SwitchADC(Switch):
    def __init__(self, name, adc_input, flags=0):
        super(SwitchADC, self).__init__(name, Switch.TYPE_ADC, flags)
        self.adc_input = adc_input


class SwitchCustom(Switch):
    def __init__(self, name, type, flags=0):
        super(SwitchCustom, self).__init__(name, type, flags)


def AZ_seq():
    return [chr(i) for i in range(ord("A"), ord("Z") + 1)]


def eprint(*args, **kwargs):
    from sys import stderr

    print(*args, file=stderr, **kwargs)


def prefixsearch(dictionary, prefix):
    for k in dictionary.keys():
        if k.startswith(prefix):
            return k[len(prefix) :]

    return None


def get_helm_input(hw_defs, source):
    source = str(source)
    gpio_def = f"HELM_INPUT_{source}_GPIO"
    pin_def = f"HELM_INPUT_{source}_PIN"

    if gpio_def not in hw_defs or pin_def not in hw_defs:
        raise ValueError(f"Unknown or disabled Helm input source '{source}'")

    return hw_defs[gpio_def], hw_defs[pin_def]


# switches from A to Z
def parse_switches(target, hw_defs, adc_parser):
    switches = []

    def find_switch(name):
        for sw in switches:
            if name == sw.name:
                return sw
        return None

    for s in AZ_seq():
        name = f"S{s}"

        reg = f"SWITCHES_GPIO_REG_{s}"
        reg_high = f"{reg}_H"
        reg_low = f"{reg}_L"

        pin = f"SWITCHES_GPIO_PIN_{s}"
        pin_high = f"{pin}_H"
        pin_low = f"{pin}_L"

        inverted = f"SWITCHES_{s}_INVERTED"

        adc_input_name = f"SW{s}"
        custom = f"SWITCHES_{s}_"

        cfs_idx = f"SWITCHES_{s}_CFS_IDX"

        helm_input = f"HELM_SWITCH_{s}"
        helm_input_high = f"{helm_input}_HIGH"
        helm_input_low = f"{helm_input}_LOW"
        helm_default = f"{helm_input}_DEFAULT"
        helm_adc = f"HELM_ADC_SW{s}" in hw_defs
        helm_inputs = [
            name for name in (helm_input, helm_input_high, helm_input_low)
            if name in hw_defs
        ]
        if helm_default in hw_defs and not (helm_inputs or helm_adc):
            raise ValueError(
                f"Switch {name} has a Helm default without a Helm input map"
            )

        switch = None
        if helm_inputs:
            if reg in hw_defs or reg_high in hw_defs or reg_low in hw_defs:
                raise ValueError(
                    f"Switch {name} has both direct and Helm input definitions"
                )
            if helm_input in hw_defs:
                if helm_input_high in hw_defs or helm_input_low in hw_defs:
                    raise ValueError(
                        f"Switch {name} mixes two-position and three-position maps"
                    )
                gpio, pin = get_helm_input(hw_defs, hw_defs[helm_input])
                switch = Switch2POS(name, gpio, pin)
            else:
                if helm_input_high not in hw_defs or helm_input_low not in hw_defs:
                    raise ValueError(
                        f"Switch {name} requires both HIGH and LOW Helm inputs"
                    )
                gpio_high, pin_high = get_helm_input(
                    hw_defs, hw_defs[helm_input_high]
                )
                gpio_low, pin_low = get_helm_input(
                    hw_defs, hw_defs[helm_input_low]
                )
                switch = Switch3POS(
                    name, gpio_high, pin_high, gpio_low, pin_low
                )
        elif reg in hw_defs:
            # 2POS switch
            reg = hw_defs[reg]
            pin = hw_defs[pin]
            switch = Switch2POS(name, reg, pin)
        elif (reg_high in hw_defs) and (reg_low in hw_defs):
            # 3POS switch
            reg_high = hw_defs[reg_high]
            pin_high = hw_defs[pin_high]
            reg_low = hw_defs[reg_low]
            pin_low = hw_defs[pin_low]
            switch = Switch3POS(name, reg_high, pin_high, reg_low, pin_low)
        elif adc_parser.find_input(adc_input_name):
            # ADC switch
            switch = SwitchADC(name, adc_input_name)
        else:
            type = prefixsearch(hw_defs, custom)
            if type is not None:
                switch = SwitchCustom(name, type)

        if switch:
            if inverted in hw_defs:
                switch.inverted = True

            if cfs_idx in hw_defs:
                switch.is_cfs = True
                switch.cfs_idx = hw_defs[cfs_idx]
            else:
                switch.is_cfs = False

            cfg = switch_cfg_by_target(target, name)
            if cfg:
                switch.default = cfg.get("default")
                switch.display = cfg.get("display")
            else:
                switch.default = "NONE"

            if helm_inputs or helm_adc:
                if helm_default in hw_defs:
                    default = hw_defs[helm_default]
                else:
                    default = (
                        "2POS" if switch.type == Switch.TYPE_2POS else "3POS"
                    )
                if default not in ("2POS", "3POS", "TOGGLE", "NONE"):
                    raise ValueError(
                        f"Switch {name} has invalid Helm default '{default}'"
                    )
                switch.default = default

            switches.append(switch)

    for i in range(1, 6 + 1):
        f_sw_marker = f"FUNCTION_SWITCH_{i}"
        if f_sw_marker in hw_defs:
            switch = find_switch(hw_defs[f_sw_marker])
            if switch:
                switch.name = f"SW{i}"
            else:
                switch = SwitchCustom(f"SW{i}", Switch.TYPE_2POS)
                switches.append(switch)

    return switches
