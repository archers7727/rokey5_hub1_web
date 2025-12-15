import { ReactNode, HTMLAttributes } from 'react'

interface CardProps extends HTMLAttributes<HTMLDivElement> {
  variant?: 'default' | 'outlined' | 'elevated'
  selectable?: boolean
  selected?: boolean
  children: ReactNode
}

export const Card = ({
  variant = 'default',
  selectable = false,
  selected = false,
  className = '',
  children,
  ...props
}: CardProps) => {
  const baseStyles = 'rounded-lg p-6 transition-all'

  const variantStyles = {
    default: 'bg-white',
    outlined: 'bg-white border-2 border-gray-200',
    elevated: 'bg-white shadow-lg'
  }

  const selectableStyles = selectable
    ? 'cursor-pointer hover:shadow-md hover:scale-[1.02]'
    : ''

  const selectedStyles = selected
    ? 'ring-2 ring-blue-500 border-blue-500 shadow-md'
    : ''

  const classes = `${baseStyles} ${variantStyles[variant]} ${selectableStyles} ${selectedStyles} ${className}`.trim()

  return (
    <div className={classes} {...props}>
      {children}
    </div>
  )
}
