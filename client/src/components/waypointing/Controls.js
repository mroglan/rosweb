import { useState } from "react";
import SettingsIcon from '@mui/icons-material/Settings';
import AddIcon from '@mui/icons-material/Add';
import EditIcon from '@mui/icons-material/Edit';
import ExitToAppIcon from '@mui/icons-material/ExitToApp';
import { Box } from "@mui/material";
import { BlueIconButtonGroup } from "../misc/buttons";

const tabOptions = [
    {value: 'settings', icon: <SettingsIcon fontSize="large" />},
    {value: 'add', icon: <AddIcon fontSize="large" />},
    {value: 'edit', icon: <EditIcon fontSize="large" />},
    {value: 'export', icon: <ExitToAppIcon fontSize="large" />}
]

export default function Controls({ws}) {

    const [tab, setTab] = useState("settings")

    return (
        <Box>
            <Box>
                <BlueIconButtonGroup options={tabOptions} selected={tab} setSelected={setTab} />
            </Box>
        </Box>
    )
}