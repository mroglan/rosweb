import { Box, Grid } from "@mui/material";
import Controls from "./Controls";

export default function Main({ws}) {

    return (
        <Box>
            <Grid container spacing={1} wrap="nowrap" minWidth={1300}>
                <Grid item width={500}>
                    <Controls ws={ws} />
                </Grid>
                <Grid item minWidth={800} flex={1}>
                    <Box bgcolor="red">
                        other
                    </Box>
                </Grid>
            </Grid>
        </Box>
    )
}