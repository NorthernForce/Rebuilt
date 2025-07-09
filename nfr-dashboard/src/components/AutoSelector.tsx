import { FormControl, MenuItem, Select } from "@mui/material";

import { useEntry } from "@frc-web-components/react";

/* AutoSelectorProps */
interface AutoSelectorProps {
    sourceKey: string;
    onChange?: (value: string) => void;
}

/**
 * AutoSelector component allows users to select an option from a dropdown menu.
 * It fetches options and the active choice from a specified source.
 *
 * @param {AutoSelectorProps} props - The properties for the AutoSelector component.
 * @returns {JSX.Element} The rendered AutoSelector component.
 */
function AutoSelector({sourceKey, onChange}: AutoSelectorProps) {
    const [options] = useEntry(`${sourceKey}/options`, ['No Options Found']);
    const [activeChoice] = useEntry(`${sourceKey}/active`, '');
    const [_selectedChoice, setSelectedChoice] = useEntry(`${sourceKey}/selected`, '');

    return (
        <>
            <div className="auto-selector" >
                <FormControl fullWidth>
                    <Select>
                        {options.map((option: string, index: number) => {
                            return (<MenuItem key={index} value={option} selected={option == activeChoice} onClick={() => {
                                setSelectedChoice(option);
                                onChange?.(option);
                            }}>
                                {option}
                            </MenuItem>);
                        })}
                    </Select>
                </FormControl>
            </div>
        </>
    );
}

export default AutoSelector;