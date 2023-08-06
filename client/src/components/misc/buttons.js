import React from 'react'
import { Button, ButtonGroup, IconButton } from '@mui/material'
import { styled } from '@mui/material/styles'

export const BluePrimaryButton = styled(Button)(({theme}) => ({
    background: theme.palette.primary.main,
    color: '#fff',
    borderRadius: theme.spacing(1),
    padding: `${theme.spacing(2)} ${theme.spacing(3)}`,
    transition: 'background 300ms',
    '&:hover': {
        background: theme.palette.primary.dark
    }
}))

export const BlueDensePrimaryButton = styled(Button)(({theme}) => ({
    background: theme.palette.primary.main,
    color: '#fff',
    borderRadius: theme.spacing(1),
    padding: `${theme.spacing(1)} ${theme.spacing(2)}`,
    transition: 'background 300ms',
    '&:hover': {
        background: theme.palette.primary.dark
    }
}))

export const BluePrimaryIconButton = styled(IconButton)(({theme}) => ({
    color: theme.palette.primary.main,
    transition: 'color 300ms',
    '&:hover': {
        color: theme.palette.primary.dark
    }
}))

export const BlueSelectedIconButton = styled(IconButton)(({theme}) => ({
    color: '#fff',
    backgroundColor: theme.palette.primary.main,
    '&:hover': {
        color: '#fff',
        backgroundColor: theme.palette.primary.main,
    }
}))

export const BlueSecondaryIconButton = styled(IconButton)(({theme}) => ({
    color: '#000',
    transition: 'color 300ms',
    '&:hover': {
        color: theme.palette.primary.dark
    }
}))

export const RedPrimaryIconButton = styled(IconButton)(({theme}) => ({
    color: theme.palette.error.main,
    transition: 'color 300ms',
    '&:hover': {
        color: theme.palette.error.dark
    }
}))

export function BlueIconButtonGroup({options, selected, setSelected}) {

    return (
        <ButtonGroup sx={{borderTopLeftRadius: 15, borderTopRightRadius: 15, overflow: 'hidden'}}>
            {options.map(option => {
                if (option.value === selected) {
                    return (
                        <BlueSelectedIconButton key={option.value} sx={{borderRadius: 0}}>
                            {option.icon}
                        </BlueSelectedIconButton>
                    )
                } else {
                    return (
                        <BluePrimaryIconButton key={option.value} onClick={() => setSelected(option.value)}
                            sx={{borderRadius: 0}}>
                            {option.icon}
                        </BluePrimaryIconButton>
                    )
                }
            })}
        </ButtonGroup>
    )
}