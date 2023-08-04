import { useState } from "react";
import SettingsIcon from '@mui/icons-material/Settings';
import AddIcon from '@mui/icons-material/Add';
import EditIcon from '@mui/icons-material/Edit';
import ExitToAppIcon from '@mui/icons-material/ExitToApp';
import { Box } from "@mui/material";
import { BlueIconButtonGroup } from "../misc/buttons";
import Settings from "./Settings";

const tabOptions = [
    {value: 'settings', icon: <SettingsIcon fontSize="large" />},
    {value: 'add', icon: <AddIcon fontSize="large" />},
    {value: 'edit', icon: <EditIcon fontSize="large" />},
    {value: 'export', icon: <ExitToAppIcon fontSize="large" />}
]

export default function Controls({ws, controls, setControls}) {

    const [tab, setTab] = useState('settings')

    return (
        <Box>
            <Box>
                <BlueIconButtonGroup options={tabOptions} selected={tab} setSelected={setTab} />
            </Box>
            <Box>
                {tab == 'settings' ? <Settings ws={ws} controls={controls} setControls={setControls} /> : ''}
            </Box>
        </Box>
    )
}