/**
 * 카드 컴포넌트
 */
import { ReactNode, HTMLAttributes } from 'react'
import './Card.css'

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
  const classes = [
    'card',
    `card-${variant}`,
    selectable ? 'card-selectable' : '',
    selected ? 'card-selected' : '',
    className,
  ]
    .filter(Boolean)
    .join(' ')

  return (
    <div className={classes} {...props}>
      {children}
    </div>
  )
}
