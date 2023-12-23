use embedded_graphics::{
    geometry::{Point, Size},
    mono_font::{ascii::FONT_4X6, MonoTextStyle},
    pixelcolor::BinaryColor,
    primitives::Rectangle,
    Drawable,
};
use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use sh1106::{interface::DisplayInterface, mode::GraphicsMode};

pub(crate) struct Screen<DI: DisplayInterface> {
    display: GraphicsMode<DI>,
}
impl<DI: DisplayInterface> Screen<DI> {
    pub fn new(mut display: GraphicsMode<DI>) -> Result<Self, DI::Error> {
        display.init()?;
        display.flush()?;
        Ok(Self { display })
    }
    pub fn write(&mut self, input: &str) -> Result<(), DI::Error> {
        self.display.clear();
        let character_style = MonoTextStyle::new(&FONT_4X6, BinaryColor::On);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Left)
            .build();
        let bounds = Rectangle::new(Point::zero(), Size::new(128, 0));
        let text_box = TextBox::with_textbox_style(input, bounds, character_style, textbox_style);

        text_box.draw(&mut self.display).unwrap();
        self.display.flush()?;

        Ok(())
    }
}
